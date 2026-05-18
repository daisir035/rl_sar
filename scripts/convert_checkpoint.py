#!/usr/bin/env python3
"""
Convert a training checkpoint (.pt) to a TorchScript inference model.

Usage:
    python3 scripts/convert_checkpoint.py policy/0315/my_policy/model_19999.pt

Outputs a TorchScript .pt file alongside the original.
"""

import os
import sys
import torch
import torch.nn as nn


class Actor(nn.Module):
    def __init__(self, num_obs=57, num_actions=16):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(num_obs, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, num_actions),
        )

    def forward(self, x):
        return self.mlp(x)


def convert(checkpoint_path, output_path=None):
    ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=False)

    if not isinstance(ckpt, dict) or "actor_state_dict" not in ckpt:
        print(f"Error: {checkpoint_path} is not a training checkpoint (no actor_state_dict)")
        return False

    actor_sd = ckpt["actor_state_dict"]

    # Infer dimensions from state dict
    num_obs = actor_sd["mlp.0.weight"].shape[1]
    num_actions = actor_sd["mlp.6.weight"].shape[0]

    print(f"Model: {num_obs} obs -> {num_actions} actions")

    # Build model and load weights (skip distribution.std_param)
    model = Actor(num_obs, num_actions)
    model_sd = {k: v for k, v in actor_sd.items() if k.startswith("mlp.")}
    model.load_state_dict(model_sd, strict=False)
    model.eval()

    # Export as TorchScript
    scripted = torch.jit.script(model)

    if output_path is None:
        base, ext = os.path.splitext(checkpoint_path)
        output_path = f"{base}_torchscript{ext}"

    torch.jit.save(scripted, output_path)
    print(f"Saved TorchScript model: {output_path}")
    return True


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <checkpoint.pt> [output.pt]")
        sys.exit(1)

    checkpoint = sys.argv[1]
    output = sys.argv[2] if len(sys.argv) > 2 else None

    if not os.path.isfile(checkpoint):
        print(f"Error: file not found: {checkpoint}")
        sys.exit(1)

    success = convert(checkpoint, output)
    sys.exit(0 if success else 1)
