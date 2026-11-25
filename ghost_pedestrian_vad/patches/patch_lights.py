#!/usr/bin/env python3
"""Make the 0.9.15 scenario_runner tolerate the 0.9.13 LightManager API.

Only touches my own copy under Work/XXL. Idempotent.
"""
import io
import os
import re

p = os.path.expanduser(
    "~/Work/XXL/vad_ghost/code/Bench2Drive/scenario_runner/srunner/scenariomanager/lights_sim.py")
src = io.open(p, encoding="utf-8").read()

if "CARLA0913_COMPAT" in src:
    print("already patched")
    raise SystemExit(0)

# guard every self._light_manager.<method>(...) call that may not exist in 0.9.13
pat = re.compile(r"^(\s*)self\._light_manager\.(\w+)\((.*)\)\s*$", re.M)


def repl(m):
    indent, meth, args = m.group(1), m.group(2), m.group(3)
    return ("%sif hasattr(self._light_manager, \"%s\"):  # CARLA0913_COMPAT\n"
            "%s    self._light_manager.%s(%s)" % (indent, meth, indent, meth, args))


new, n = pat.subn(repl, src)
io.open(p, "w", encoding="utf-8").write(new)
print("patched calls:", n)
