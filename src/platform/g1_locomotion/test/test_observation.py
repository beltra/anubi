"""Offline observation-dimension tests for the AMO policy runtime.

These tests need torch + the vendored assets. They are SKIPPED automatically if
torch is not importable, so the package's other tests (flake8/pep257) still run
on a host without torch. The forward-pass assertion additionally requires CUDA
because amo_jit.pt carries a CUDA-resident constant (see amo_policy_runtime.py).
"""

import pytest

torch = pytest.importorskip('torch')

from g1_locomotion.amo_policy_runtime import (  # noqa: E402
    AmoPolicy,
    default_robot_state,
)


def _make_policy():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    return AmoPolicy(device=device), device


def test_reconstructed_observation_dim():
    """The reconstructed observation must be exactly 1043 (amo_jit input dim)."""
    policy, _ = _make_policy()
    state = default_robot_state(policy.default_dof_pos)
    policy.reset_yaw(float(state['rpy'][2]))
    obs, extra = policy.build_observation(state)
    assert obs.shape[0] == 1043
    assert obs.shape[0] == policy.expected_obs_dim
    import numpy as np
    assert np.array(extra).flatten().shape[0] == 2325
    assert np.array(extra).flatten().shape[0] == policy.expected_extra_hist_dim


@pytest.mark.skipif(not torch.cuda.is_available(),
                    reason='amo_jit.pt forward pass requires CUDA')
def test_forward_pass_and_arms_held():
    """One forward pass returns a 15-DoF action; arms stay at default."""
    import numpy as np
    policy, _ = _make_policy()
    state = default_robot_state(policy.default_dof_pos)
    policy.reset_yaw(float(state['rpy'][2]))
    obs, extra = policy.build_observation(state)
    pd_target, raw_action = policy.infer(obs, extra)
    assert np.shape(raw_action) == (15,)
    assert pd_target.shape == (29,)
    # Arms (15:29) held at default_dof_pos.
    assert np.allclose(pd_target[15:], policy.default_dof_pos[15:])
