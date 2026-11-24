#!/usr/bin/env python3
"""Build multi-view demo videos from GhostPedestrianV4 runs.

Layout (1280x720):
  top-left   chase camera      (960x540 -> 640x360)
  top-right  top-down camera   (800x450  -> 640x360)
  bottom-left LEAD front camera (384x384 -> 360x360, padded)
  bottom-right HUD text (speed / brake / occlusion / LEAD walker score)

Frames are re-timed by CARLA frame id, not by file order: missing frame ids are
filled with the previous image so playback stays at the real 20 Hz sim rate.
A burned-in subtitle shows, per 0.1 s, the ego speed, brake, the pedestrian's
occluded fraction from LEAD's front camera and LEAD's own walker detection
score (joined from pred.jsonl by sim frame).
"""
import json
import os
import shutil
import subprocess

RUNS = "/root/autodl-tmp/ghost_runs_v4"
OUT = "/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian_v4/demos"
TMP = "/tmp/v4demo"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FPS = 20
W, H = 1280, 720

DEMOS = [
    ("v4_demo_occluder_only", "demo_occluder_only",
     "Parked SUV only - no pedestrian (scene baseline)"),
    ("v4_demo_collision", "demo_collision",
     "LEAD - occluded pedestrian at 20 m - COLLISION"),
    ("v4_demo_safe", "demo_safe",
     "LEAD - same scene, 40 m trigger - SAFE"),
    ("v4_demo_oracle", "demo_oracle",
     "Oracle brake at first visibility - SAFE"),
]


def manifest(run):
    path = os.path.join(RUNS, run, "frames_manifest.jsonl")
    by = {}
    if not os.path.exists(path):
        return by
    for line in open(path):
        try:
            rec = json.loads(line)
        except Exception:
            continue
        by.setdefault(rec["tag"], {})[int(rec["frame"])] = rec.get("t")
    return by


def trace(run):
    path = os.path.join(RUNS, run, "trace.jsonl")
    out = {}
    if not os.path.exists(path):
        return out
    for line in open(path):
        try:
            r = json.loads(line)
        except Exception:
            continue
        if r.get("frame") is not None:
            out[int(r["frame"])] = r
    return out


def pred(run):
    path = os.path.join(RUNS, run, "pred.jsonl")
    out = {}
    if not os.path.exists(path):
        return out
    for line in open(path):
        try:
            r = json.loads(line)
        except Exception:
            continue
        f = r.get("frame")
        if f is None:
            continue
        out[int(f)] = r
    return out


def _ok_png(path):
    """True only if the file ends with the PNG IEND chunk (a truncated write
    from image.save_to_disk at process exit leaves no IEND)."""
    try:
        with open(path, "rb") as f:
            f.seek(-8, os.SEEK_END)
            return f.read() == b"IEND\xae\x42\x60\x82"
    except Exception:
        return False


def align(run, work):
    by = manifest(run)
    if not by.get("lead"):
        return [], {}
    tags = [t for t in ("lead", "top", "chase") if by.get(t)]
    starts = {t: min(by[t]) for t in tags}
    first = min(starts.values())
    last = max(max(by[t]) for t in tags)
    keep = [f for f in range(first, last + 1) if f in by["lead"]]
    if not keep:
        return [], {}
    shutil.rmtree(work, ignore_errors=True)
    for t in tags:
        os.makedirs(os.path.join(work, t), exist_ok=True)
    stats = {t: {"gaps": 0, "filled": 0, "n": 0} for t in tags}
    for t in tags:
        last_img = None
        for i, fid in enumerate(keep):
            src = os.path.join(RUNS, run, "frames", t, "%06d.png" % fid)
            dst = os.path.join(work, t, "%05d.png" % (i + 1))
            if os.path.exists(src) and _ok_png(src):
                if last_img is not None and fid - last_img[0] > 1:
                    stats[t]["gaps"] += 1
                last_img = (fid, src)
                shutil.copy(src, dst)
                stats[t]["n"] += 1
            elif last_img is not None:
                shutil.copy(last_img[1], dst)
                stats[t]["filled"] += 1
    return keep, stats


def _ts(t):
    h = int(t // 3600)
    m = int((t % 3600) // 60)
    s = t % 60
    return "%d:%02d:%05.2f" % (h, m, s)


def ass(run, outname, label, keep, work):
    """Build an ASS overlay: static title + 10 Hz metric HUD."""
    tr = trace(run)
    pr = pred(run)
    first = keep[0]
    path = os.path.join(work, "hud.ass")
    dur = len(keep) / float(FPS)
    head = (
        "[Script Info]\nScriptType: v4.00+\nPlayResX: %d\nPlayResY: %d\n"
        "WrapStyle: 2\nScaledBorderAndShadow: yes\n\n"
        "[V4+ Styles]\n"
        "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, "
        "OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, "
        "ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, "
        "Alignment, MarginL, MarginR, MarginV, Encoding\n"
        "Style: Title,DejaVu Sans,24,&H00FFFFFF,&H00FFFFFF,&H00000000,"
        "&H99000000,-1,0,0,0,100,100,0,0,3,2,0,7,16,16,16,1\n"
        "Style: Hud,DejaVu Sans Mono,22,&H00FFFFFF,&H00FFFFFF,&H00000000,"
        "&H99000000,0,0,0,0,100,100,0,0,3,2,0,7,16,16,16,1\n\n"
        "[Events]\nFormat: Layer, Start, End, Style, Name, MarginL, MarginR, "
        "MarginV, Effect, Text\n"
        % (W, H))
    lines = [head]
    lines.append("Dialogue: 0,0:00:00.00,%s,Title,,0,0,0,,{\\pos(16,12)}%s\n"
                 % (_ts(max(dur, 0.1)), label.replace(",", " ")))
    step = 2  # every 2 frames = 10 Hz
    for i in range(0, len(keep), step):
        fid = keep[i]
        t0 = i / float(FPS)
        t1 = min(dur, (i + step) / float(FPS))
        r = tr.get(fid)
        p = pr.get(fid)
        if r and r.get("t") is not None:
            rows = ["t_sim %6.2f s   frame %d" % (r["t"], fid)]
        else:
            rows = ["video %6.2f s   frame %d" % (t0, fid)]
        if r:
            rows.append("ego  %5.1f km/h   brake %.2f  thr %.2f"
                        % (r.get("ego_speed", 0.0) * 3.6, r.get("brake", 0.0),
                           r.get("throttle", 0.0)))
            occ = r.get("occlusion")
            fov = r.get("in_fov")
            if occ is None:
                rows.append("ped  (no pedestrian)")
            elif fov is not None and fov < 0.5:
                rows.append("ped  out of cam FOV   occ=%.2f" % occ)
            elif occ >= 0.5:
                rows.append("ped  HIDDEN by SUV    occ=%.2f" % occ)
            else:
                rows.append("ped  VISIBLE          occ=%.2f" % occ)
            rows.append("dist ego-ped %s m"
                        % ("--" if r.get("dist_ego_walker") is None
                           else "%.1f" % r["dist_ego_walker"]))
        if p is not None:
            rows.append("LEAD walker %.2f  bev %.3f  tgt %.1f"
                        % (p.get("walker_score_max") or 0.0,
                           p.get("bev_walker_max") or 0.0,
                           p.get("target_speed_scalar") or 0.0))
        body = "\\N".join(rows)
        lines.append("Dialogue: 0,%s,%s,Hud,,0,0,0,,{\\pos(656,378)}%s\n"
                     % (_ts(t0), _ts(t1), body))
    with open(path, "w") as f:
        f.writelines(lines)
    return path


def build(run, outname, label):
    work = os.path.join(TMP, outname)
    keep, stats = align(run, work)
    if not keep:
        print("SKIP %s: no aligned frames" % run)
        return False
    os.makedirs(OUT, exist_ok=True)
    sub = ass(run, outname, label, keep, work)
    desc = " ".join("%s n=%d gaps=%d filled=%d"
                    % (t, s["n"], s["gaps"], s["filled"])
                    for t, s in sorted(stats.items()))
    print("%s: %s (%d timeline frames, %.1fs)"
          % (run, desc, len(keep), len(keep) / float(FPS)))

    # 4 cells of 640x360: chase | top / lead | hud
    fc = ("[0:v]scale=640:360,setsar=1[c1];"
          "[1:v]scale=640:360,setsar=1[c2];"
          "[2:v]scale=360:360,pad=640:360:140:0:color=black,setsar=1[c3];"
          "color=c=black:s=640x360:d=%.3f[c4];"
          "[c1][c2]hstack=inputs=2[r1];[c3][c4]hstack=inputs=2[r2];"
          "[r1][r2]vstack=inputs=2[v]" % (len(keep) / float(FPS)))
    fc += (";[v]subtitles='%s':fontsdir=/usr/share/fonts/truetype/dejavu[v2]"
           % sub.replace("'", "\\'"))
    inputs = []
    for t in ("chase", "top", "lead"):
        inputs += ["-framerate", str(FPS), "-i",
                   os.path.join(work, t, "%05d.png")]
    out = os.path.join(OUT, outname + ".mp4")
    cmd = (["ffmpeg", "-y", "-loglevel", "error"] + inputs +
           ["-filter_complex", fc, "-map", "[v2]", "-an",
            "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
            "-pix_fmt", "yuv420p", "-r", str(FPS), out])
    subprocess.check_call(cmd)
    print("made %s" % out)
    return True


def comparison():
    seq = [d for d in ("demo_occluder_only", "demo_collision", "demo_safe",
                       "demo_oracle")
           if os.path.exists(os.path.join(OUT, d + ".mp4"))]
    if len(seq) < 2:
        return
    ins = []
    for d in seq:
        ins += ["-i", os.path.join(OUT, d + ".mp4")]
    n = len(seq)
    parts = ["[%d:v]scale=640:360[s%d]" % (i, i) for i in range(n)]
    if n == 4:
        fc = (",".join(parts) + ";[s0][s1]hstack[a];[s2][s3]hstack[b];"
              "[a][b]vstack[g]")
    else:
        fc = ",".join(parts) + ";[s0][s1]hstack[g]"
    out = os.path.join(OUT, "demo_comparison.mp4")
    subprocess.check_call(["ffmpeg", "-y", "-loglevel", "error"] + ins +
                          ["-filter_complex", fc, "-map", "[g]", "-an",
                           "-c:v", "libx264", "-preset", "veryfast",
                           "-crf", "22", "-pix_fmt", "yuv420p", out])
    print("made", out)


def main():
    os.makedirs(TMP, exist_ok=True)
    for run, outname, label in DEMOS:
        build(run, outname, label)
    comparison()


if __name__ == "__main__":
    main()
