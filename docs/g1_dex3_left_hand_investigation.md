# G1 Dex3-1 Left Hand - Finger Fault Investigation

Date: 2026-07-15. Robot: Unitree G1 with Dex3-1 dexterous hands (left/right, 7 DOF each).
Reported symptom: "one part of the left hand fingers does not work."

## Bottom line (second read, after power-cycle/reseat)

The fault is a **partially-contacting flex cable on the motor-5/6 finger**, not a dead
motor. Between the two reads the state changed materially - some conductors in that
finger's cable came back, others did not.

| signal                    | first read        | after power-cycle/reseat        |
|---------------------------|-------------------|---------------------------------|
| motor[6] bus voltage      | 0.00 V (dark)     | **24.50 V** (restored)          |
| motor[6] position `q`     | 0.000 (no data)   | **-0.309** (encoder reading)    |
| motor[6] casing temp      | 0                 | **33 C** (sensor reading)       |
| motor[6] winding temp     | 0                 | **0 - still not reading**       |
| motor[6] `motorstate`     | 0x00000000        | **0x00000080 - active fault**   |
| tactile pad[4]            | dead (0 data)     | **healthy** (12/12 taxels)      |
| tactile pad[5]            | dead (0 data)     | **still dead, `lost` climbing** |

Remaining faults (stable across repeated samples):
- **`motor_state[6]` reports `motorstate = 0x00000080`** (bit 7) persistently, and its
  **winding temperature reads 0 while its casing reads 33 C**. Power, comms and the
  encoder all work now - the winding temp-sense channel does not.
- **Tactile pad[5] is still off the bus**: all-zero data with a `lost` counter
  **actively incrementing** (90,960 -> 135,804 -> 139,668 -> 143,530 across samples,
  ~3.9k per sample). Healthy pads sit at a frozen `lost = 1605`, so a *climbing* counter
  is the live-failure marker; a static one is just history since boot.
- Everything else is healthy: motors 0-5 at 24.5 V with valid temps/positions, 8 of 9
  tactile pads streaming, hand-level `error = [0, 0]`.

Reading: a multi-conductor flex cable carries motor power, motor data, winding temp-sense
and the tactile bus for that finger. After the reseat, **motor power/data and pad 4 make
contact; the winding temp-sense line and pad 5's channel still do not.** That subset
pattern is the signature of a **partially seated or damaged FFC / connector** - specific
conductors open while their neighbours conduct. It is not a failed motor (the motor is
alive and reporting) and not a controller fault (no hand-level error).

**Caution:** winding over-temperature protection keys off `temperature[1]`, which now
reads 0 on motor 6. With that sense line open, the 120 C winding protection for that motor
may be blind - avoid loading that finger until the cable is fixed.

**`mode` decoding (corrected).** In the STATE direction `mode` is a plain value -
**0x01 = control mode** (per Unitree's unitree_ros2 MotorState docs). The
`RIS_Mode_t {id:4, status:3, timeout:1}` bitfield in the SDK example applies to the
**command** direction only; decoding `state.mode` as those bitfields is wrong (an earlier
pass here did that and mis-read "id"/"status"). Corrected reading:
- motors 0-5: `mode = 0x01` (in control mode), `motorstate = 0x00000000`.
- **motor 6: `mode = 0x00` (NOT in control mode)** - an independent confirmation that it
  is the only faulted motor, alongside `motorstate = 0x80`.

Motor 6's encoder is confirmed live: `q` tracked from -0.309 to +0.681 when the finger was
moved by hand between reads. Position sensing and power are fine; the fault is the open
winding temp-sense line, the non-zero `motorstate`, and pad 5 being off the bus.

## How the codes were read (connectivity)

Initially unreachable (wired NIC `enp129s0` was down, no `192.168.123.x` address). After
the robot link was brought up, `enp129s0` came up as `192.168.123.224/24`; ARP resolves
`192.168.123.161` (robot) and `192.168.123.164`. The diagnostic then received
`rt/lf/dex3/left/state` directly on `enp129s0` (ICMP ping to the robot is filtered, but
DDS discovery and traffic work).

Command used:

```bash
python3 dex3_left_diag.py enp129s0
```

## Force / pressure sensors (live)

The left hand exposes 9 tactile pads in `HandState.press_sensor_state[]`; each pad has
`pressure[12]` + `temperature[12]` taxels, a cumulative `lost` dropped-frame counter, and
`reserve`. Read with `dex3_left_press.py enp129s0`:

First read: pads 4 and 5 both dead (`lost` ~3.8M). After the power-cycle/reseat:

- **8 of 9 pads healthy** (0,1,2,3,**4**,6,7,8): streaming all 12 taxels, temps 31-32 C,
  `lost` frozen at 1605. Pad 4 recovered. Values read at rest are raw baseline (~100k);
  squeeze a fingertip and watch that pad's peak pressure rise to confirm touch response.
- **pad[5] still dead**: all-zero pressure/temperature, `lost` **climbing** (~3.9k per
  sample) - the sensor bus is still continuously timing out on it.

Note the `lost` semantics: it is cumulative, and it **reset on the power cycle** (3.8M ->
~91k and climbing again). Compare *growth*, not absolute value - healthy pads' counters
are frozen, a faulted pad's climbs.

Correlated fault: pad 5 and motor 6's open winding-sense line belong to the **same finger**
(the motor-5/6 finger). A 9-pad / 3-finger hand plausibly carries 3 pads per finger,
putting pads 3,4,5 in one group. Exact pad->finger wiring is not documented on-disk;
confirm by squeezing each fingertip and watching which pad reacts.

## Motor LED (3 fast + 8 slow blinks)

No official Unitree table decoding the Dex3-1 hand motor blink pattern could be found
(checked Weston Robot G1 diag guide, G1 user manuals, Trossen Go1 troubleshooting, Unitree
motor SDK guide, Dex3 manuals). Unitree publishes solid-color states (blue=normal,
red=error) and battery blink codes only. So the exact code word for "3 fast + 8 slow" is
NOT assigned here - it must be confirmed with Unitree support (send a video of the pattern).

What is certain and useful:
- A structured blinking LED = the motor controller has **latched a fault** (a healthy
  motor is solid / slow heartbeat, not a repeating coded blink). Common convention is a
  fast-blink preamble followed by a slow-blink count = the error number; do not assign a
  specific fault name to the count without the vendor table.
- **The blinking motor is almost certainly motor 6.** In the first read motor 6 sat at
  0 V and could not have lit its LED at all; after the repair attempt it is powered
  (24.5 V) and is the **only** motor reporting a non-zero `motorstate` (0x00000080), so
  its controller is alive and annunciating that fault.
- **Hypothesis worth checking with Unitree (unverified):** `motorstate = 0x80` is bit 7 -
  the **8th** error bit - and the LED shows **8** slow blinks. The slow-blink count may
  simply name which error bit is set, with the 3 fast blinks as a preamble/category
  marker. The correspondence is striking but is NOT confirmed by any vendor source; do
  not treat it as fact.
- Whatever the code word, it matches the measured state: motor 6 is powered and
  communicating but has an open **winding temp-sense** line (casing 33 C, winding 0 C), and
  its finger-mate tactile pad 5 is still off the bus - one partially-contacting flex cable.

## Dex3-1 left hand layout (motor index -> joint)

From the URDF (`dexterous_hand_description/dex3_1_l.urdf`) and the SDK example limit
arrays. 7 motors, indexed 0-6 in `HandState.motor_state[]`:

| motor | joint (URDF)             | axis | limit (rad)        | finger  |
|-------|--------------------------|------|--------------------|---------|
| 0     | left_hand_thumb_0_joint  | y    | -1.047 .. +1.047   | thumb   |
| 1     | left_hand_thumb_1_joint  | z    | -0.611 .. +1.047   | thumb   |
| 2     | left_hand_thumb_2_joint  | z    |  0.000 .. +1.745   | thumb   |
| 3     | finger _0 (index/middle) | z    | -1.571 .. 0        | finger A|
| 4     | finger _1 (index/middle) | z    | -1.745 .. 0        | finger A|
| 5     | finger _0 (index/middle) | z    | -1.571 .. 0        | finger B|
| 6     | finger _1 (index/middle) | z    | -1.745 .. 0        | finger B|

- Thumb = 3 DOF (motors 0,1,2). Index and middle = 2 DOF each (motors {3,4} and {5,6}).
- Which of {3,4} vs {5,6} is index vs middle depends on Unitree's motor-ID wiring;
  confirm live by commanding one finger and seeing which `q` pair moves.
- "One part of the fingers not working" most likely maps to ONE of these motor groups
  (a whole finger = a 2-motor pair, or one thumb segment).

## Where the error codes live (`unitree_hg::HandState_`)

Per-motor, `motor_state[i]`:
- `mode` (uint8, RIS byte): bits `id[0:4]`, `status[4:7]`, `timeout[7]`.
  - `status == 0` -> motor disabled/locked (no torque). `status == 1` -> enabled (FOC).
  - `timeout == 1` -> command watchdog tripped / lost bus comms to that motor.
  - `id != i` -> the motor at that slot is reporting a different ID (wiring/enumeration).
- `motorstate` (uint32): per-motor fault/status word; **nonzero = fault**.
- `temperature[2]` (int16): `[0]` = casing, `[1]` = **winding**. Winding overtemperature
  protection cuts torque; critical winding limit is **120 C**.
- `q, dq, tau_est`: position / velocity / estimated torque - used to separate a comms
  fault (q frozen) from a mechanical jam (tau_est high, dq ~= 0).
- `vol`: per-motor bus voltage (0 / collapsed -> power or connector loss).

Hand-level: `error[2]` (uint32) nonzero -> hand controller / power / bus fault (often
affects more than one motor); `power_v/power_a/system_v/device_v` -> hand supply health.

DDS topics: command `rt/dex3/left/cmd` (`HandCmd_`), state `rt/dex3/left/state` and the
low-frequency `rt/lf/dex3/left/state` (used by the SDK example subscriber).

## Read the codes (run on the robot's onboard computer)

```bash
# iface = the robot-facing NIC that holds a 192.168.123.x address (Jetson: enP8p1s0)
python3 dex3_left_diag.py enP8p1s0
```

`dex3_left_diag.py` (in this session's scratchpad) subscribes to
`rt/lf/dex3/left/state`, decodes the RIS mode bits, `motorstate`, casing/winding
temperature, `q/dq/tau_est/vol` for all 7 motors, and prints a per-finger verdict that
flags exactly which motor group is faulted.

Alternative (official) sanity check: the prebuilt
`/tools/unitree_sdk2/build/bin/g1_dex3_example <iface>` -> choose `L`, press `p` to
print live joint state (won't decode error words, but confirms which joints move).

## Diagnosis decision tree (from the live readout)

1. Affected motor(s) show `timeout == 1`, or `motorstate != 0`, or `q`/`temperature`
   frozen / `vol == 0`  -> **comms / connector loss to that finger.** Most common cause of
   one dead finger: a loose/worn flex cable (FFC) at the finger or palm PCB. Fix: power
   cycle the hand; reseat the finger's connector; inspect the cable.
2. Affected motor `temperature[1]` near/above ~100-120 C, `tau_est` collapses to ~0,
   finger goes limp under load -> **winding overtemperature lockout.** Fix: let it cool,
   then lower holding gains (`kp/kd`) and command rate; avoid long high-torque holds.
3. Affected motor `status == 1` (enabled), `tau_est` saturates, but `dq ~= 0` and `q`
   won't track -> **mechanical jam / tendon or gear fault** in that finger. Fix: power
   off, inspect the finger for obstruction/damage, check range of motion by hand.
4. State fully nominal but the finger still won't move on command -> **command-path
   issue**: `kp == 0` for that motor, or the command isn't populating that motor index,
   or `status` left at 0. Fix: verify the `HandCmd` you send sets `mode.status = 1` and
   nonzero `kp` for those indices.
5. Hand-level `error[2] != 0` or supply voltages sagging -> **hand power/bus fault**;
   escalate to a full hand power cycle and, if persistent, Unitree support.

Ranked most-likely for "one finger dead": (1) connector/FFC comms loss, then (3)
mechanical jam, then (2) overtemperature (usually intermittent/limp rather than fully
dead), then (4) command-path.

## Restore connectivity (so codes can be read)

The canonical path is to run the hand tools **on the robot's onboard Jetson**, which owns
the `192.168.123.x` robot LAN. If you must drive the hand from this dev host:
1. Physically connect this host's wired port to the robot LAN (cable is already present on
   `enp129s0`: `carrier=1`).
2. Bring the NIC up with a static address on the robot subnet, e.g. `192.168.123.224`
   (host IP already used for the Mid-360 on this subnet), netmask `/24`.
3. `ping -c2 192.168.123.161` must reply, then run `dex3_left_diag.py enp129s0`.
   Note: Humble CycloneDDS may need the NIC pinned via the deprecated
   `<NetworkInterfaceAddress>` (see reference `docker/config/cyclonedds_jetson.xml`) to
   avoid SEDP poisoning from a remote host.

## Sources

- Unitree Dex3-1 product / docs: support.unitree.com/home/en/dex3-1_hand,
  support.unitree.com/home/en/G1_developer
- SDK interface (this host): `unitree_sdk2` `example/g1/dex3/g1_dex3_example.cpp`,
  `unitree_hg` `HandState.msg` / `MotorState.msg`, `dexterous_hand_description/dex3_1_l.urdf`
- unitree_sdk2 reference (DeepWiki): temperature[0]=casing, temperature[1]=winding,
  winding critical limit 120 C.
- Overheat failure mode: github.com/unitreerobotics/unitree_sdk2_python/issues/129
