# Ghost Pedestrian demo videos

The scenario attaches an ego-mounted RGB camera (640x360, 90° FOV) and saves
every 5th simulation frame into `<run>/frames/`. These videos are assembled from
those frames at 10 fps with `scripts/make_demos.sh`. **They are scenario
monitoring renders, not LEAD's actual input sensors** — LEAD's camera rig and
LiDAR are separate and are logged separately by the P3.5 hook.

| File | Run | What it shows |
|---|---|---|
| `demo_occluder_only.mp4` | `occluder_only` | Parked vehicle only, no pedestrian; ego passes normally (control). |
| `demo_safe_25m.mp4` | `gp_25m` | Pedestrian released at 25 m; LEAD brakes and yields; no collision. |
| `demo_collision_24m.mp4` | `gp_24m` | Pedestrian released at 24 m; LEAD arrives too early; front-corner contact. |
| `demo_oracle_brake.mp4` | `gp_24m_or_visible` | Same 24 m scene, but an external full brake is applied at first geometric visibility; no collision. |
| `demo_comparison.mp4` | 2x2 grid | The four clips side by side, first 6 s each. |

Reading order: occluder-only → 25 m safe → 24 m collision → Oracle brake.
The important comparison is the last two: identical scene and release state,
different control, different outcome.

Rebuild on AutoDL with:

```bash
bash /root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian/scripts/make_demos.sh
```
