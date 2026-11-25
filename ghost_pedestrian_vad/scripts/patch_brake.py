#!/usr/bin/env python3
"""Env-gated brake-hold / hysteresis on top of VAD's bang-bang controller."""
import io
import os

p = os.path.expanduser(
    "~/Work/XXL/vad_ghost/code/Bench2DriveZoo/team_code/vad_b2d_agent.py")
src = io.open(p, encoding="utf-8").read()

if "VAD_BRAKE_HOLD_S" in src:
    print("already patched")
    raise SystemExit(0)

anchor = """        if throttle_traj > brake_traj: brake_traj = 0.0
"""
assert anchor in src, "anchor not found"

block = """        if throttle_traj > brake_traj: brake_traj = 0.0

        # --- opt-in brake-hold / hysteresis (identical when unset) ---
        _hold = float(os.environ.get("VAD_BRAKE_HOLD_S", "0") or 0)
        _hyst = float(os.environ.get("VAD_BRAKE_HYST", "0") or 0)
        if _hold > 0 or _hyst > 0:
            _now = float(self.step) / 20.0
            _des = float(metadata_traj.get("desired_speed", 0.0))
            _v = float(tick_data["speed"])
            _prev = bool(getattr(self, "_gp_brake_prev", False))
            if _prev and brake_traj <= 0.0:
                _held = (_hold > 0) and ((_now - float(getattr(self, "_gp_brake_onset", -1e9))) < _hold)
                _hyst_on = (_hyst > 0) and _des > 0.01 and (_v / _des) > _hyst
                if _held or _hyst_on:
                    brake_traj = 1.0
                    throttle_traj = 0.0
            _on = brake_traj > 0.3
            if _on and not _prev:
                self._gp_brake_onset = _now
            self._gp_brake_prev = _on
"""

src = src.replace(anchor, block, 1)
io.open(p, "w", encoding="utf-8").write(src)
print("patched brake hold/hysteresis")
