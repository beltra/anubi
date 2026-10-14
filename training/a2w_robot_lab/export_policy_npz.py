#!/usr/bin/env python3
"""Convert an rsl_rl checkpoint (model_<iter>.pt) into the .npz a2w_policy_controller loads.

Writes the actor MLP as W0, b0, W1, b1, ... plus a JSON metadata string with the joint
order and action/observation scales, so the Gazebo controller can reject a mismatched
policy. Requires torch (run it in the Isaac Lab env).

    python export_policy_npz.py logs/rsl_rl/unitree_a2w_flat/<run>/model_4999.pt a2w_flat.npz
"""

import argparse
import json
import re

import numpy as np
import torch

LEG_JOINTS = [
    leg + "_" + part + "_joint"
    for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")
]
WHEEL_JOINTS = [leg + "_wheel_joint" for leg in ("FR", "FL", "RR", "RL")]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("checkpoint")
    parser.add_argument("output")
    args = parser.parse_args()

    state = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    weights = state.get("model_state_dict", state)
    if any(k.startswith("actor_obs_normalizer") for k in weights):
        raise SystemExit("observation normalization is not supported by the numpy runtime")

    layer_ids = sorted({int(m.group(1)) for k in weights for m in [re.match(r"actor\.(\d+)\.weight$", k)] if m})
    if not layer_ids:
        raise SystemExit("no actor.<n>.weight tensors in " + args.checkpoint)
    arrays = {}
    for i, layer in enumerate(layer_ids):
        arrays["W%d" % i] = weights["actor.%d.weight" % layer].numpy().astype(np.float32)
        arrays["b%d" % i] = weights["actor.%d.bias" % layer].numpy().astype(np.float32)

    metadata = {
        "source": args.checkpoint,
        "leg_joints": LEG_JOINTS,
        "wheel_joints": WHEEL_JOINTS,
        "activation": "elu",
        "obs_dim": int(arrays["W0"].shape[1]),
        "act_dim": int(arrays["W%d" % (len(layer_ids) - 1)].shape[0]),
    }
    np.savez(args.output, metadata=json.dumps(metadata), **arrays)
    print("wrote %s: %s" % (args.output, json.dumps({k: metadata[k] for k in ("obs_dim", "act_dim")})))


if __name__ == "__main__":
    main()
