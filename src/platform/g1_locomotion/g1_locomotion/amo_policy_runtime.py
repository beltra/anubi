"""
AMO policy runtime for the Unitree G1 (legs-only, navigation).

This is a clean, standalone re-implementation of the Psi0 AMO inference path.
It loads the three vendored TorchScript assets and reproduces, byte-for-byte,
the observation that the Psi0 stack feeds to ``amo_jit.pt`` -- but with all of
the teleop / arm-IK / Pinocchio / hands machinery stripped out. Arms are held
at their default pose; only the 15-DoF lower body (12 legs + 3 waist) is driven
by the policy.

Provenance of every magic number is cited inline to the exact Psi0 source file
and line so the port can be audited against the original.

Origin files (read-only, NOT a runtime dependency of this package):
  - Psi0/real/teleop/teleop/master_whole_body.py
      RobotTaskmaster.__init__ (constants, default_dof_pos, adapter load) and
      RobotTaskmaster.get_ik_observation (the observation builder).
  - Psi0/real/teleop/teleop/robot_control/robot_body_ik.py
      G1_29_BodyIK.solve_whole_body_ik (policy call, action scaling, is_teleop).

--------------------------------------------------------------------------------
OBSERVATION LAYOUT (verified empirically against amo_jit.pt -- see README gate)
--------------------------------------------------------------------------------
Two indexing constants drive everything:

  obs_idx = np.r_[0:19, 22:26]                       # 23 elements
      i.e. joints 0..18 (legs+waist+left arm incl. left wrists) and 22..25
      (right shoulder x3 + right elbow). The RIGHT wrists (26,27,28) and the
      gap 19..21 vs 22 are how Psi0 drops the wrist DoFs from the model input
      ("no wrist joint", master_whole_body.py:134). NOTE this 23-vector still
      INCLUDES the left wrists (19,20,21); only the right wrists are excluded.
      -> obs_dof has 23 entries. obs_dof[15:] is the 8-element "demo dof"
         (left shoulder x3 + left elbow, right shoulder x3 + right elbow).

  n_proprio = 3 + 2 + 2 + 23*3 + 2 + 15 = 93         # master_whole_body.py:134

obs_prop (93)  = concat, in this exact order:
    [ 0: 3)  ang_vel * scales_ang_vel        (3)   gyro * 0.25
    [ 3: 5)  rpy[:2]                          (2)   roll, pitch (NO yaw)
    [ 5: 7)  (sin(dyaw), cos(dyaw))           (2)   heading error vs target_yaw
    [ 7:30)  (obs_dof_pos - obs_default)      (23)  joint pos error
    [30:53)  obs_dof_vel * scales_dof_vel     (23)  joint vel * 0.05
    [53:76)  obs_last_action                  (23)  last policy action (raw)
    [76:78)  gait_obs = sin(gait_cycle*2pi)   (2)   gait phase
    [78:93)  adapter_output                   (15)  adapter network output

obs_demo (17)  = master_whole_body.py:154-156, 518-527:
    [0:8)   obs_dof_pos[15:]   (8)   the 8 demo dof (overwrites template)
    [8]     vx                 (1)   forward velocity command
    [9]     vy                 (1)   lateral velocity command
    [10]    0.0                (1)   (template default; never written -> 0)
    [11]    torso_yaw          (1)
    [12]    torso_pitch        (1)
    [13]    torso_roll         (1)
    [14:17) torso_height x3    (3)

obs_priv (3)   = zeros(n_priv), n_priv=3                 master_whole_body.py:133
obs_hist (930) = proprio_history_buf (history_len=10) flattened = 10*93 = 930

observation = concat(obs_prop, obs_demo, obs_priv, obs_hist)
            = 93 + 17 + 3 + 930 = 1043   <-- amo_jit.pt obs input dim (VERIFIED)

The SECOND policy argument is the privileged/extra history:
  extra_hist = extra_history_buf (extra_history_len=25) flattened
             = 25 * 93 = 2325                            <-- VERIFIED
  (NOTE: this uses extra_history_len=25, NOT history_len=10. The two history
   buffers are distinct deques fed the same obs_prop each tick.)

amo_jit.pt(observation[1043], extra_hist[2325]) -> raw_action[15]

These dims were confirmed by loading amo_jit.pt and running a forward pass:
obs=1043 / ext=2325 succeeds and returns shape (1,15); obs=293 (the other
candidate from prior analyses) fails. See README "OBSERVATION VERIFICATION GATE".
--------------------------------------------------------------------------------
"""

from __future__ import annotations

import os
from collections import deque

import numpy as np

# torch is imported lazily inside AmoPolicy.__init__ so that this module can be
# imported (e.g. for unit tests of pure helpers) on a host without torch.


def _asset_dir() -> str:
    """Locate the vendored *.pt assets.

    Prefers the colcon-installed share/g1_locomotion/assets (set via the
    G1_LOCOMOTION_ASSETS env var by a launch file if desired), then falls back
    to the in-source assets/ directory next to this module.
    """
    env = os.environ.get('G1_LOCOMOTION_ASSETS')
    if env and os.path.isdir(env):
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'assets')


# ---------------------------------------------------------------------------
# Default robot state. AmoPolicy.build_observation() consumes a plain dict so
# the policy is decoupled from the DDS layer (robot_dds_io.py fills the dict;
# the offline self-test fills it with these defaults).
# ---------------------------------------------------------------------------
def default_robot_state(default_dof_pos: np.ndarray) -> dict:
    """A synthetic 'standing at the default pose, level, still' robot state.

    Used by the offline self-test and as the seed before the first real
    lowstate arrives. All quantities are in the Psi0 conventions.
    """
    return {
        # 29-DoF measured joint position / velocity (motor order, idx 0..28).
        'motorstate': default_dof_pos.astype(np.float32).copy(),
        'velstate': np.zeros(29, dtype=np.float32),
        # Body angular velocity (gyro), rad/s, base frame.
        'ang_vel': np.zeros(3, dtype=np.float32),
        # Base orientation as roll/pitch/yaw (rad). Yaw is absolute; the policy
        # only sees rpy[:2] (roll, pitch) plus a heading-error term (dyaw).
        'rpy': np.zeros(3, dtype=np.float32),
    }


class AmoPolicy:
    """Loads the AMO assets and turns a robot-state dict into a 15-DoF target.

    Lifecycle::

        policy = AmoPolicy(device="cpu")
        policy.reset_yaw(state["rpy"][2])              # once, at hand-off
        ...
        target29 = policy.step(state, vx, vy, yaw)     # per control tick
        # target29[:15]  = policy-driven legs+waist
        # target29[15:]  = arms held at default_dof_pos[15:]

    ``step`` reproduces get_ik_observation + solve_whole_body_ik(is_teleop=False).
    """

    # --- constants, verbatim from Psi0 master_whole_body.py ----------------
    NJ = 29                       # master_whole_body.py:132
    N_PRIV = 3                    # master_whole_body.py:133  (obs_priv = zeros(3))
    N_PROPRIO = 3 + 2 + 2 + 23 * 3 + 2 + 15   # = 93        master_whole_body.py:134
    HISTORY_LEN = 10              # master_whole_body.py:135  (obs_hist deque)
    EXTRA_HISTORY_LEN = 25        # master_whole_body.py:136  (extra_hist deque)
    N_DEMO_DOF = 8                # master_whole_body.py:137  (4+4, no wrist)

    SCALES_ANG_VEL = 0.25         # master_whole_body.py:129
    SCALES_DOF_VEL = 0.05         # master_whole_body.py:130
    ACTION_SCALE = 0.25           # master_whole_body.py:152 / robot_body_ik.py:30
    DT = 0.02                     # master_whole_body.py:163  (50 Hz policy tick)

    GAIT_FREQ = 1.3               # master_whole_body.py:167
    CONTROL_DT = 0.02             # master_whole_body.py:168

    # default_dof_pos, 29-DoF, master_whole_body.py:139-145 (== robot_body_ik.py:32-38)
    #   legs(12): L hip p/r/y, knee, ankle p/r ; R hip p/r/y, knee, ankle p/r
    #   waist(3): yaw, roll, pitch
    #   left arm(7): shoulder p/r/y, elbow, wrist r/p/y
    #   right arm(7): shoulder p/r/y, elbow, wrist r/p/y
    DEFAULT_DOF_POS = np.array([
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
        0.0, 0.0, 0.0,
        0.5, 0.0, 0.2, 0.3, 0.0, 0.0, 0.0,
        0.5, 0.0, -0.2, 0.3, 0.0, 0.0, 0.0,
    ], dtype=np.float64)

    # The 23-element model joint subset: 0..18 then 22..25 (master_whole_body.py:479).
    # Drops the right wrists (26,27,28) and the index gap 19..21->22 keeps only the
    # left wrists. Built once at class scope.
    OBS_IDX = np.r_[0:19, 22:26]

    # Action raw clip applied before scaling (robot_body_ik.py:454).
    ACTION_CLIP = 40.0

    def __init__(self, device: str | None = None, assets_dir: str | None = None):
        import torch  # lazy
        self._torch = torch
        if device is None:
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.device = device

        # NOTE(asset limitation): amo_jit.pt was TorchScript-traced with a
        # constant tensor baked onto cuda:0 (see save_jit.py origin in the
        # traceback). As a result the policy forward pass ONLY runs on a CUDA
        # device -- on CPU it raises "Expected all tensors to be on the same
        # device ... cuda:0 ... cpu". The observation builder and dimension
        # checks are device-agnostic; only AmoPolicy.infer() needs CUDA.
        # TODO(verify): re-export amo_jit.pt with map_location-clean constants
        # if CPU inference is ever required.

        adir = assets_dir or _asset_dir()
        amo_path = os.path.join(adir, 'amo_jit.pt')
        adapter_path = os.path.join(adir, 'adapter_jit.pt')
        stats_path = os.path.join(adir, 'adapter_norm_stats.pt')
        for p in (amo_path, adapter_path, stats_path):
            if not os.path.isfile(p):
                raise FileNotFoundError(f'AMO asset not found: {p}')

        # --- policy + adapter (TorchScript) ---  master_whole_body.py:183-186,
        #     robot_body_ik.py:27-28.
        self.policy = torch.jit.load(amo_path, map_location=self.device)
        self.policy.eval()
        self.adapter = torch.jit.load(adapter_path, map_location=self.device)
        self.adapter.eval()
        for param in self.adapter.parameters():
            param.requires_grad = False

        # --- adapter normalization stats ---  master_whole_body.py:188-192.
        norm_stats = torch.load(stats_path, weights_only=False)
        self.input_mean = torch.tensor(
            norm_stats['input_mean'], device=self.device, dtype=torch.float32)
        self.input_std = torch.tensor(
            norm_stats['input_std'], device=self.device, dtype=torch.float32)
        self.output_mean = torch.tensor(
            norm_stats['output_mean'], device=self.device, dtype=torch.float32)
        self.output_std = torch.tensor(
            norm_stats['output_std'], device=self.device, dtype=torch.float32)

        # --- mutable controller state ---
        self.default_dof_pos = self.DEFAULT_DOF_POS.copy()
        self.last_action = np.zeros(self.NJ)              # master_whole_body.py:151
        self.gait_cycle = np.array([0.25, 0.25])          # master_whole_body.py:166
        self._in_place_stand_flag = True                  # master_whole_body.py:165

        # Yaw tracking (heading-error term in obs). master_whole_body.py:158-161.
        self.target_yaw = 0.0
        self.yaw_offset = 0.0
        self._yaw_initialized = False
        self.dyaw = 0.0

        # Torso command. For navigation this is held at the "stand" values that
        # Psi0's maintain_standing() writes every tick (master_whole_body.py:757-760):
        # height 0.75 m, level. The adapter turns this into the 15-d output that
        # the policy uses as a whole-body posture reference.
        self.torso_height = 0.75
        self.torso_roll = 0.0
        self.torso_pitch = 0.0
        self.torso_yaw = 0.0

        # demo_obs_template (17-vector). master_whole_body.py:154-156.
        self.demo_obs_template = np.zeros((self.N_DEMO_DOF + 3 + 3 + 3,))
        self.demo_obs_template[:self.N_DEMO_DOF] = \
            self.default_dof_pos[np.r_[15:19, 22:26]]
        self.demo_obs_template[self.N_DEMO_DOF + 6:self.N_DEMO_DOF + 9] = 0.75

        # History buffers, pre-filled with zeros. master_whole_body.py:176-181.
        self.proprio_history_buf = deque(maxlen=self.HISTORY_LEN)
        self.extra_history_buf = deque(maxlen=self.EXTRA_HISTORY_LEN)
        for _ in range(self.HISTORY_LEN):
            self.proprio_history_buf.append(np.zeros(self.N_PROPRIO))
        for _ in range(self.EXTRA_HISTORY_LEN):
            self.extra_history_buf.append(np.zeros(self.N_PROPRIO))

        # Command (set per-tick by the driver). master_whole_body.py:122-124.
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

        # Cached last observation (for introspection / the self-test).
        self.observation = None
        self.extra_hist = None

    # -----------------------------------------------------------------------
    @property
    def expected_obs_dim(self) -> int:
        """Reconstructed observation dimension (must equal amo_jit input dim)."""
        # obs_prop(93) + obs_demo(17) + obs_priv(3) + obs_hist(10*93=930) = 1043
        obs_demo = self.N_DEMO_DOF + 3 + 3 + 3
        return (self.N_PROPRIO + obs_demo + self.N_PRIV
                + self.HISTORY_LEN * self.N_PROPRIO)

    @property
    def expected_extra_hist_dim(self) -> int:
        """Reconstructed privileged-history dimension (2nd policy argument)."""
        return self.EXTRA_HISTORY_LEN * self.N_PROPRIO

    # -----------------------------------------------------------------------
    def reset_yaw(self, current_yaw: float) -> None:
        """Latch the yaw offset to the current heading (Psi0 reset_yaw_offset).

        Mirrors master_whole_body.py:619-621: the first time we see a heading we
        treat it as zero so dyaw starts at 0. Also resets target_yaw so the robot
        does not immediately try to rotate.
        """
        self.yaw_offset = float(current_yaw)
        self.target_yaw = 0.0
        self.dyaw = 0.0
        self._yaw_initialized = True

    # -----------------------------------------------------------------------
    def build_observation(self, state: dict):
        """Byte-exact port of RobotTaskmaster.get_ik_observation(record=True).

        ``state`` keys: 'motorstate'(29), 'velstate'(29), 'ang_vel'(3),
        'rpy'(3). self.vx/vy/vyaw and self.torso_* must already be set.

        Returns (observation[1043], extra_hist_deque).
        """
        torch = self._torch
        motorstate = np.asarray(state['motorstate'], dtype=np.float32)
        velstate = np.asarray(state['velstate'], dtype=np.float32)
        ang_vel = np.asarray(state['ang_vel'], dtype=np.float32)
        rpy = np.asarray(state['rpy'], dtype=np.float32)

        if not self._yaw_initialized:
            # Defensive: behave like reset_yaw on first ever call.
            self.reset_yaw(float(rpy[2]))

        # --- heading error (dyaw) --- master_whole_body.py:459-468 (record=True)
        self.target_yaw += self.vyaw * self.DT
        dyaw = rpy[2] - self.yaw_offset - self.target_yaw
        dyaw = np.remainder(dyaw + np.pi, 2 * np.pi) - np.pi
        if self._in_place_stand_flag:
            dyaw = 0.0
        self.dyaw = dyaw

        # --- joint subset --- master_whole_body.py:479-486
        obs_idx = self.OBS_IDX
        obs_dof_vel = velstate[obs_idx].copy()
        # Zero the velocity of the dropped/uncontrolled joints (ankles + waist).
        # Indices into the 23-vector: master_whole_body.py:481.
        obs_dof_vel[[4, 5, 10, 11, 13, 14]] = 0.0
        obs_dof_pos = motorstate[obs_idx]
        obs_default_dof_pos = self.default_dof_pos[obs_idx]
        obs_last_action = self.last_action[obs_idx]

        # --- gait phase --- master_whole_body.py:488
        gait_obs = np.sin(self.gait_cycle * 2 * np.pi)

        # --- adapter forward pass --- master_whole_body.py:490-501
        # adapter input = [torso_height, torso_yaw, torso_pitch, torso_roll,
        #                  obs_dof_pos[15:]  (the 8 demo dof)]  -> 12 elements.
        adapter_input_np = np.concatenate([np.zeros(4), obs_dof_pos[15:]])
        adapter_input_np[0] = self.torso_height
        adapter_input_np[1] = self.torso_yaw
        adapter_input_np[2] = self.torso_pitch
        adapter_input_np[3] = self.torso_roll
        adapter_input = torch.tensor(adapter_input_np).to(
            self.device, dtype=torch.float32).unsqueeze(0)
        adapter_input = (adapter_input - self.input_mean) / (self.input_std + 1e-8)
        with torch.no_grad():
            adapter_output = self.adapter(adapter_input.view(1, -1))
            adapter_output = adapter_output * self.output_std + self.output_mean
        adapter_output_np = adapter_output.cpu().numpy().squeeze()

        # --- obs_prop (93) --- master_whole_body.py:503-513
        obs_prop = np.concatenate([
            ang_vel * self.SCALES_ANG_VEL,
            rpy[:2],
            (np.sin(self.dyaw), np.cos(self.dyaw)),
            (obs_dof_pos - obs_default_dof_pos),
            obs_dof_vel * self.SCALES_DOF_VEL,
            obs_last_action,
            gait_obs,
            adapter_output_np,
        ])

        # --- obs_priv / obs_hist --- master_whole_body.py:515-516
        obs_priv = np.zeros((self.N_PRIV,))
        obs_hist = np.array(self.proprio_history_buf).flatten()

        # --- obs_demo (17) --- master_whole_body.py:518-527
        obs_demo = self.demo_obs_template.copy()
        obs_demo[:self.N_DEMO_DOF] = obs_dof_pos[15:]
        obs_demo[self.N_DEMO_DOF] = self.vx
        obs_demo[self.N_DEMO_DOF + 1] = self.vy
        # Update the in-place-stand flag exactly as Psi0 does (line 522). This
        # gates dyaw to 0 while ~stationary, and the gait cycle below.
        self._in_place_stand_flag = (
            (np.abs(self.vx) < 0.1)
            and (np.abs(self.vy) < 0.1)
            and (np.abs(self.vyaw) < 0.1)
        )
        obs_demo[self.N_DEMO_DOF + 3] = self.torso_yaw
        obs_demo[self.N_DEMO_DOF + 4] = self.torso_pitch
        obs_demo[self.N_DEMO_DOF + 5] = self.torso_roll
        obs_demo[self.N_DEMO_DOF + 6:self.N_DEMO_DOF + 9] = self.torso_height

        # Push this tick's proprio into BOTH history buffers (line 529-530).
        self.proprio_history_buf.append(obs_prop)
        self.extra_history_buf.append(obs_prop)

        # --- assemble --- master_whole_body.py:532
        observation = np.concatenate((obs_prop, obs_demo, obs_priv, obs_hist))
        extra_hist = self.extra_history_buf

        # --- advance gait cycle --- master_whole_body.py:535-539
        self.gait_cycle = np.remainder(
            self.gait_cycle + self.CONTROL_DT * self.GAIT_FREQ, 1.0)
        if self._in_place_stand_flag and (
                (np.abs(self.gait_cycle[0] - 0.25) < 0.05)
                or (np.abs(self.gait_cycle[1] - 0.25) < 0.05)):
            self.gait_cycle = np.array([0.25, 0.25])
        if (not self._in_place_stand_flag) and (
                (np.abs(self.gait_cycle[0] - 0.25) < 0.05)
                and (np.abs(self.gait_cycle[1] - 0.25) < 0.05)):
            self.gait_cycle = np.array([0.25, 0.75])

        self.observation = observation
        self.extra_hist = extra_hist
        return observation, extra_hist

    # -----------------------------------------------------------------------
    def infer(self, observation: np.ndarray, extra_hist):
        """Run the policy. Port of solve_whole_body_ik(is_teleop=False).

        robot_body_ik.py:448-457. Returns (pd_target_29, raw_action_15).
        """
        torch = self._torch
        obs_tensor = torch.from_numpy(
            observation).float().unsqueeze(0).to(self.device)
        with torch.no_grad():
            extra = torch.tensor(
                np.array(extra_hist).flatten().copy(),
                dtype=torch.float).view(1, -1).to(self.device)
            raw_action = self.policy(obs_tensor, extra).cpu().numpy().squeeze()

        raw_action = np.clip(raw_action, -self.ACTION_CLIP, self.ACTION_CLIP)  # :454
        scaled_actions = raw_action * self.ACTION_SCALE                        # :455

        # pd_target = [scaled(15), zeros(14)] + default_dof_pos  (robot_body_ik.py:457)
        # i.e. the policy drives the first 15 DoF; arms (15:29) stay at default.
        pd_target = np.concatenate(
            [scaled_actions, np.zeros(14)]) + self.default_dof_pos
        return pd_target, raw_action

    # -----------------------------------------------------------------------
    def step(self, state: dict, vx: float, vy: float, vyaw: float):
        """Full per-tick path: set command -> build obs -> infer -> update last_action.

        Returns the 29-DoF PD position target. Indices 0..14 are the policy
        output (legs + waist); 15..28 are held at default_dof_pos (arms).
        """
        self.vx = float(vx)
        self.vy = float(vy)
        self.vyaw = float(vyaw)

        observation, extra_hist = self.build_observation(state)
        pd_target, raw_action = self.infer(observation, extra_hist)

        # last_action update -- master_whole_body.py:780 / :891. The first 15
        # entries are the raw policy action; the last 14 reconstruct the "action"
        # that would have produced the measured arm pose (arms are IK-driven in
        # teleop; here arms are static so this term stays ~0).
        motorstate = np.asarray(state['motorstate'], dtype=np.float64)
        self.last_action = np.concatenate([
            raw_action.copy(),
            (motorstate - self.default_dof_pos)[15:] / self.ACTION_SCALE,
        ])
        return pd_target
