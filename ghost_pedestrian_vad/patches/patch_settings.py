#!/usr/bin/env python3
"""Make WorldSettings construction work on CARLA 0.9.13 (no spectator_as_ego)."""
import io
import os

p = os.path.expanduser(
    "~/Work/XXL/vad_ghost/code/Bench2Drive/leaderboard/leaderboard/leaderboard_evaluator.py")
src = io.open(p, encoding="utf-8").read()

old = '''                settings = carla.WorldSettings(
                    synchronous_mode = True,
                    fixed_delta_seconds = 1.0 / self.frame_rate,
                    deterministic_ragdolls = True,
                    spectator_as_ego = False
                )
'''
new = '''                settings = carla.WorldSettings(
                    synchronous_mode = True,
                    fixed_delta_seconds = 1.0 / self.frame_rate,
                    deterministic_ragdolls = True,
                )
                if hasattr(settings, "spectator_as_ego"):
                    settings.spectator_as_ego = False
'''
if new in src or "hasattr(settings, \"spectator_as_ego\")" in src:
    print("already patched")
    raise SystemExit(0)
assert old in src, "anchor not found"
src = src.replace(old, new, 1)
io.open(p, "w", encoding="utf-8").write(src)
print("patched settings")
