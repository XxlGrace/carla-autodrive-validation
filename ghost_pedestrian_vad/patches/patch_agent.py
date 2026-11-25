#!/usr/bin/env python3
"""Patch the VAD agent (my copy) with an env-gated JSONL trace, idempotently."""
import io
import os
import sys

path = os.path.expanduser(
    "~/Work/XXL/vad_ghost/code/Bench2DriveZoo/team_code/vad_b2d_agent.py")
src = io.open(path, encoding="utf-8").read()

if "VAD_TRACE_JSONL" in src:
    print("already patched")
    sys.exit(0)

anchor = "        self.prev_control = control\n"
if anchor not in src:
    print("ANCHOR NOT FOUND")
    sys.exit(1)

hook = '''        _trace = os.environ.get("VAD_TRACE_JSONL")
        if _trace:
            try:
                with open(_trace, "a") as _f:
                    _f.write(json.dumps({
                        "frame": int(input_data["CAM_FRONT"][0]),
                        "step": int(self.step),
                        "t": round(self.step / 20.0, 3),
                        "speed": round(float(tick_data["speed"]), 3),
                        "desired_speed": round(float(self.pid_metadata.get("desired_speed", -1.0)), 3),
                        "steer": round(float(control.steer), 3),
                        "throttle": round(float(control.throttle), 3),
                        "brake": round(float(control.brake), 3),
                        "plan_end": [round(float(v), 2) for v in out_truck[-1]],
                    }) + "\\n")
            except Exception:
                pass
'''

src = src.replace(anchor, hook + anchor, 1)
io.open(path, "w", encoding="utf-8").write(src)
print("patched", path)
