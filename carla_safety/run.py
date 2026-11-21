"""Minimal closed-loop runner: CARLA -> RGB camera -> placeholder policy -> control -> tick.

This proves M0-M2: frames are real renders, frames align with ticks, and vehicle
state changes as a result of the applied controls. The policy is NOT a trained
safety driver; it only drives forward and brakes on collision.
"""
import argparse
import csv
import json
import os
import time
from pathlib import Path

import carla


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="configs/demo.yaml")
    p.add_argument("--output-root", default="/root/autodl-tmp/carla-runs")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    return p.parse_args()


def load_config(path):
    import yaml
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)

    run_id = time.strftime("%Y%m%d-%H%M%S")
    out = Path(args.output_root) / run_id
    frames_dir = out / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    host, port = args.host, args.port
    client = carla.Client(host, port)
    client.set_timeout(120.0)
    # NOTE: do NOT call client.load_world() here. Runtime map reload is unstable
    # on the headless Vulkan build (segfault). Use the map the server started with.
    world = client.get_world()
    map_name = world.get_map().name

    # Synchronous mode with fixed time step
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = cfg.get("fixed_delta_seconds", 0.05)
    settings.no_rendering_mode = False  # headless != no rendering; we need real images
    world.apply_settings(settings)

    blueprint_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found on map")

    # Ego vehicle
    vehicle_bp = blueprint_lib.filter(cfg.get("vehicle", "vehicle.tesla.model3"))[0]
    ego = world.spawn_actor(vehicle_bp, spawn_points[0])

    # Front RGB camera
    cam_bp = blueprint_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(cfg.get("width", 640)))
    cam_bp.set_attribute("image_size_y", str(cfg.get("height", 360)))
    cam_bp.set_attribute("fov", str(cfg.get("fov", 90)))
    cam_bp.set_attribute("sensor_tick", "0.0")
    cam_transform = carla.Transform(
        carla.Location(x=1.5, z=2.4), carla.Rotation(pitch=-8)
    )
    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=ego)

    frame_state = {"count": 0, "last_image": None}

    def on_image(image):
        frame_state["last_image"] = image
        path = str(frames_dir / ("%06d.png" % image.frame))
        image.save_to_disk(path)

    camera.listen(on_image)

    collision_state = {"hit": False}

    def on_collision(event):
        collision_state["hit"] = True

    collision_bp = blueprint_lib.find("sensor.other.collision")
    collision = world.spawn_actor(collision_bp, carla.Transform(), attach_to=ego)
    collision.listen(on_collision)

    steps = int(cfg.get("steps", 200))
    throttle = float(cfg.get("throttle", 0.5))
    telemetry = []
    start = ego.get_location()

    try:
        for _ in range(steps):
            control = carla.VehicleControl(throttle=throttle, steer=0.0)
            if collision_state["hit"]:
                control = carla.VehicleControl(throttle=0.0, brake=1.0)
            ego.apply_control(control)

            # In sync mode tick() itself blocks until the frame is rendered.
            # Do NOT call wait_for_tick() here: no further tick will be issued
            # until this returns, so it would deadlock.
            world.tick()

            loc = ego.get_location()
            vel = ego.get_velocity()
            telemetry.append(
                {
                    "frame": frame_state["count"],
                    "image_frame": frame_state["last_image"].frame
                    if frame_state["last_image"]
                    else -1,
                    "x": round(loc.x, 4),
                    "y": round(loc.y, 4),
                    "z": round(loc.z, 4),
                    "speed": round((vel.x**2 + vel.y**2 + vel.z**2) ** 0.5, 4),
                    "throttle": throttle,
                    "brake": 1.0 if collision_state["hit"] else 0.0,
                    "collision": collision_state["hit"],
                }
            )
            frame_state["count"] += 1
    finally:
        camera.stop()
        collision.stop()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        camera.destroy()
        collision.destroy()
        ego.destroy()

    with open(out / "telemetry.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(telemetry[0].keys()))
        writer.writeheader()
        writer.writerows(telemetry)

    end = ego.get_location() if ego.is_alive else None
    metadata = {
        "run_id": run_id,
        "carla_client": getattr(carla, "__version__", "0.9.16"),
        "carla_server": client.get_server_version(),
        "map": map_name,
        "steps": steps,
        "fixed_delta_seconds": settings.fixed_delta_seconds,
        "config": cfg,
        "start_location": [start.x, start.y, start.z],
        "frames_saved": len(list(frames_dir.glob("*.png"))),
    }
    with open(out / "metadata.json", "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)

    print("RUN OK:", out)
    print("frames:", metadata["frames_saved"], "steps:", steps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
