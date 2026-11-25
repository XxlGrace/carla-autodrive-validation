#!/usr/bin/env python3
"""Guard traffic-manager calls that do not exist in CARLA 0.9.13.

Bench2Drive's background traffic uses TM methods added in 0.9.14+.
Only my own copy under Work/XXL is touched.
"""
import io
import os
import re

paths = [
    "~/Work/XXL/vad_ghost/code/Bench2Drive/scenario_runner/srunner/scenarios/background_activity.py",
]
pat = re.compile(r"^(\s*)self\._tm\.(\w+)\((.*)\)\s*$", re.M)

for p in paths:
    p = os.path.expanduser(p)
    src = io.open(p, encoding="utf-8").read()
    if "CARLA0913_TM" in src:
        print("already patched", p)
        continue

    def repl(m):
        indent, meth, args = m.group(1), m.group(2), m.group(3)
        return ("%sif hasattr(self._tm, \"%s\"):  # CARLA0913_TM\n"
                "%s    self._tm.%s(%s)" % (indent, meth, indent, meth, args))

    new, n = pat.subn(repl, src)
    io.open(p, "w", encoding="utf-8").write(new)
    print("patched %d TM calls in %s" % (n, os.path.basename(p)))
