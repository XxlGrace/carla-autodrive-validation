#!/usr/bin/env python3
"""Move the JSONL hook from setup() into run_step() (it matched the first
`self.prev_control = control`)."""
import io
import os

p = os.path.expanduser(
    "~/Work/XXL/vad_ghost/code/Bench2DriveZoo/team_code/vad_b2d_agent.py")
src = io.open(p, encoding="utf-8").read()

start = src.find('        _trace = os.environ.get("VAD_TRACE_JSONL")')
end_marker = '        self.prev_control = control\n'
if start == -1:
    print("hook not found")
    raise SystemExit(0)
end = src.find(end_marker, start)
if end == -1:
    print("hook end not found")
    raise SystemExit(0)
hook = src[start:end]
src = src[:start] + src[end:]

if "VAD0913_HOOK2" in src:
    print("already moved")
    raise SystemExit(0)

hook = hook.replace('_trace = os.environ.get("VAD_TRACE_JSONL")',
                    '_trace = os.environ.get("VAD_TRACE_JSONL")  # VAD0913_HOOK2')

anchor = '        self.prev_control = control\n        \n        if len(self.prev_control_cache)==10:'
if anchor not in src:
    # fallback: second occurrence of the assignment
    first = src.find(end_marker)
    second = src.find(end_marker, first + 1)
    if second == -1:
        print("run_step anchor not found")
        raise SystemExit(1)
    src = src[:second] + hook + src[second:]
else:
    src = src.replace(anchor, hook + anchor, 1)

io.open(p, "w", encoding="utf-8").write(src)
print("hook moved into run_step")
