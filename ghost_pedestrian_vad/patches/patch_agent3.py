#!/usr/bin/env python3
"""RoadOption is an enum here; the agent treats the route command as int."""
import io
import os

p = os.path.expanduser(
    "~/Work/XXL/vad_ghost/code/Bench2DriveZoo/team_code/vad_b2d_agent.py")
src = io.open(p, encoding="utf-8").read()

if "VAD0913_ROADOPTION" in src:
    print("already patched")
    raise SystemExit(0)

old = """        command = tick_data['command_near']
        if command < 0:
            command = 4
        command -= 1
"""
new = """        command = tick_data['command_near']
        if hasattr(command, 'value'):  # VAD0913_ROADOPTION
            command = int(command.value)
        if command < 0:
            command = 4
        command -= 1
"""
assert old in src, "anchor not found"
src = src.replace(old, new, 1)
io.open(p, "w", encoding="utf-8").write(src)
print("patched RoadOption -> int")
