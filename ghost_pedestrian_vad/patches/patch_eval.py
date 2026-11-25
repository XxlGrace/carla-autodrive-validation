#!/usr/bin/env python3
"""Let the evaluator reuse an externally started CARLA (0.9.13 headless quirks).

Env-gated with VAD_EXTERNAL_CARLA so the file stays usable without it.
Only touches my own copy.
"""
import io
import os

p = os.path.expanduser(
    "~/Work/XXL/vad_ghost/code/Bench2Drive/leaderboard/leaderboard/leaderboard_evaluator.py")
src = io.open(p, encoding="utf-8").read()

if "VAD_EXTERNAL_CARLA" in src:
    print("already patched")
    raise SystemExit(0)

old = '''        self.carla_path = os.environ["CARLA_ROOT"]
        args.port = find_free_port(args.port)
        cmd1 = f"{os.path.join(self.carla_path, 'CarlaUE4.sh')} -RenderOffScreen -nosound -carla-rpc-port={args.port} -graphicsadapter={args.gpu_rank}"
        self.server = subprocess.Popen(cmd1, shell=True, preexec_fn=os.setsid)
        print(cmd1, self.server.returncode, flush=True)
        atexit.register(os.killpg, self.server.pid, signal.SIGKILL)
        time.sleep(30)
'''
new = '''        self.carla_path = os.environ["CARLA_ROOT"]
        if os.environ.get("VAD_EXTERNAL_CARLA"):
            print("VAD_EXTERNAL_CARLA: reusing CARLA on port %s" % args.port, flush=True)
            self.server = None
            time.sleep(3)
        else:
            args.port = find_free_port(args.port)
            cmd1 = f"{os.path.join(self.carla_path, 'CarlaUE4.sh')} -RenderOffScreen -nosound -carla-rpc-port={args.port} -graphicsadapter={args.gpu_rank}"
            self.server = subprocess.Popen(cmd1, shell=True, preexec_fn=os.setsid)
            print(cmd1, self.server.returncode, flush=True)
            atexit.register(os.killpg, self.server.pid, signal.SIGKILL)
            time.sleep(30)
'''
assert old in src, "setup anchor not found"
src = src.replace(old, new, 1)

old2 = '''        if crashed:
            cmd2 = "ps -ef | grep '-graphicsadapter="+ str(args.gpu_rank) + "' | grep -v grep | awk '{print $2}' | xargs -r kill -9"
'''
new2 = '''        if crashed and not os.environ.get("VAD_EXTERNAL_CARLA"):
            cmd2 = "ps -ef | grep '-graphicsadapter="+ str(args.gpu_rank) + "' | grep -v grep | awk '{print $2}' | xargs -r kill -9"
'''
assert old2 in src, "cleanup anchor not found"
src = src.replace(old2, new2, 1)

io.open(p, "w", encoding="utf-8").write(src)
print("patched", p)
