#!/usr/bin/env python3
"""Remove the bev camera / IS_BENCH2DRIVE dependency in my copy of the agent.

- IS_BENCH2DRIVE=False path references an undefined `now`; replace with the
  save name derived from the config string (as the True path does).
- The bev camera at z=50 m violates the leaderboard's 3 m extrinsics limit.
"""
import io
import os

p = os.path.expanduser(
    "~/Work/XXL/vad_ghost/code/Bench2DriveZoo/team_code/vad_b2d_agent.py")
src = io.open(p, encoding="utf-8").read()

if "VAD0913_SAVENAME" in src:
    print("already patched")
    raise SystemExit(0)

old = """        if IS_BENCH2DRIVE:
            self.save_name = path_to_conf_file.split('+')[-1]
        else:
            self.save_name = '_'.join(map(lambda x: '%02d' % x, (now.month, now.day, now.hour, now.minute, now.second)))
"""
new = """        # VAD0913_SAVENAME: `now` is undefined on the non-Bench2Drive path
        self.save_name = path_to_conf_file.split('+')[-1]
"""
assert old in src, "save_name anchor not found"
src = src.replace(old, new, 1)

old2 = """        if IS_BENCH2DRIVE:
            sensors += [
"""
new2 = """        if IS_BENCH2DRIVE and os.environ.get("VAD_USE_BEV", "") == "1":
            sensors += [
"""
assert old2 in src, "sensors anchor not found"
src = src.replace(old2, new2, 1)

# tick() reads input_data['bev'] unconditionally; make it optional
old3 = """        bev = cv2.cvtColor(input_data['bev'][1][:, :, :3], cv2.COLOR_BGR2RGB)
"""
new3 = """        if 'bev' in input_data:
            bev = cv2.cvtColor(input_data['bev'][1][:, :, :3], cv2.COLOR_BGR2RGB)
        else:
            bev = np.zeros((512, 512, 3), dtype=np.uint8)
"""
assert old3 in src, "bev anchor not found"
src = src.replace(old3, new3, 1)

io.open(p, "w", encoding="utf-8").write(src)
print("patched agent (save_name / bev optional)")
