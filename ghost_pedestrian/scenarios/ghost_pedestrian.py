#!/usr/bin/env python
"""GhostPedestrian v0 - minimal occluded pedestrian crossing for LEAD eval.

Design (v0, Town01, fixed sunny daytime):
  * Ego is driven by LEAD (unchanged); this module only spawns actors, drives the
    pedestrian and records metrics, all inside the evaluator-owned world/tick.
  * A parked vehicle on the roadside occludes a pedestrian. When the ego enters a
    configurable trigger distance of the crossing point, the pedestrian dashes
    across the ego lane at a fixed speed.
  * The trigger distance is the sweep variable. Ego camera frames, a geometric
    occlusion flag (cast_ray) and per-tick telemetry are saved for analysis.

Activated by a route XML like:
  <scenario type="GhostPedestrian" name="GP_25m">
    <trigger_point x="396.3" y="260" z="1.0" yaw="270"/>
    <ghost_params trigger_distance="25" walker_speed="1.5" crossing_ahead="52"
                  output_dir="/root/autodl-tmp/ghost_runs/gp_25m" seed="0"/>
  </scenario>
"""
from __future__ import print_function

import json
import math
import os

import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    AtomicBehavior,
    ActorDestroy,
    WaitForever,
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToLocation,
    DriveDistance,
)
from srunner.scenarios.basic_scenario import BasicScenario


GP_TAG = "GP"  # log prefix


def _log(msg):
    print("[{}] {}".format(GP_TAG, msg), flush=True)


def _flat_dist(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


class _GhostState:
    """Shared state between sensor callbacks and py_trees behaviors."""

    def __init__(self, out_dir, save_frames=True, frame_every=3, max_frames=400):
        self.out_dir = out_dir
        self.frames_dir = os.path.join(out_dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)
        self.trace_path = os.path.join(out_dir, "trace.jsonl")
        # Truncate any stale trace from a previous run, then append every record
        # immediately so data survives an evaluator timeout / crash.
        with open(self.trace_path, "w"):
            pass
        with open(os.path.join(out_dir, "summary.json"), "w") as f:
            json.dump({"status": "running"}, f)
        self.save_frames = save_frames
        self.frame_every = frame_every
        self.max_frames = max_frames
        self.frame = 0
        self.saved = 0
        self.records = []
        self.collisions = []
        self.released = False
        self.release_time = None
        self.brake_onset = None
        self.min_dist = float("inf")
        self.min_dist_time = None
        self.stop = False
        self.dumped = False

    def camera_cb(self, image):
        if not self.save_frames or self.stop or self.saved >= self.max_frames:
            return
        self.frame += 1
        if self.frame % self.frame_every == 0:
            self.saved += 1
            image.save_to_disk(os.path.join(self.frames_dir, "%04d.png" % self.frame))

    def collision_cb(self, event):
        other = event.other_actor.type_id if event.other_actor else "unknown"
        self.collisions.append({"t": round(GameTime.get_time(), 3), "other": other})

    def append_record(self, record):
        """Persist each record immediately (crash-safe) and keep in memory."""
        self.records.append(record)
        with open(self.trace_path, "a") as f:
            f.write(json.dumps(record) + "\n")
            f.flush()

    def dump(self):
        if self.dumped:
            return
        self.dumped = True
        self.stop = True

        ped_hits = [c for c in self.collisions if "walker" in c["other"]]
        first_brake_rel = None
        if self.release_time is not None and self.brake_onset is not None:
            first_brake_rel = round(self.brake_onset - self.release_time, 3)

        # Occlusion evidence: was the walker geometrically hidden by the car
        # during the approach (before release)?
        pre = [r for r in self.records if not r["released"] and r.get("occluded") is not None]
        occ_frac = None
        if pre:
            occ_frac = round(sum(1 for r in pre if r["occluded"]) / float(len(pre)), 3)

        summary = {
            "status": "final",
            "num_records": len(self.records),
            "collisions_all": self.collisions,
            "collision_pedestrian": len(ped_hits) > 0,
            "min_distance_m": None if self.min_dist == float("inf") else round(self.min_dist, 3),
            "min_distance_time_s": self.min_dist_time,
            "release_time_s": self.release_time,
            "brake_onset_s": self.brake_onset,
            "brake_onset_after_release_s": first_brake_rel,
            "occluded_fraction_before_release": occ_frac,
            "occluded_samples": len(pre),
            "frames_saved": self.saved,
        }
        with open(os.path.join(self.out_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        _log("dumped {} records, {} frames, ped_collision={}, occ_frac={}".format(
            len(self.records), self.saved, summary["collision_pedestrian"], occ_frac))


class DashWalker(AtomicBehavior):
    """Deterministically steps a walker along a straight line each tick.

    KeepVelocity uses WalkerControl, which does nothing when simulate_physics
    is disabled (needed for deterministic spawning). Setting the transform
    directly avoids that dependency entirely.
    """

    def __init__(self, state, walker, start, end, speed, name="DashWalker"):
        super(DashWalker, self).__init__(name)
        self.state = state
        self.walker = walker
        self.start = start
        self.end = end
        self.speed = float(speed)
        self.total = _flat_dist(start, end)

    def update(self):
        # Absolute time since release: robust even when py_trees re-initialises
        # this behaviour (which otherwise teleports the walker back to start).
        now = GameTime.get_time()
        release = self.state.release_time if self.state.release_time is not None else now
        elapsed = max(0.0, now - release)
        frac = min(1.0, elapsed * self.speed / self.total) if self.total > 0 else 1.0
        loc = carla.Location(
            self.start.x + (self.end.x - self.start.x) * frac,
            self.start.y + (self.end.y - self.start.y) * frac,
            self.start.z + (self.end.z - self.start.z) * frac,
        )
        tf = self.walker.get_transform()
        tf.location = loc
        self.walker.set_transform(tf)
        if frac >= 1.0:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ReleasePedestrian(AtomicBehavior):
    """Marks the moment the pedestrian starts crossing (ped becomes visible)."""

    def __init__(self, state, name="ReleasePedestrian"):
        super(ReleasePedestrian, self).__init__(name)
        self.state = state

    def update(self):
        if not self.state.released:
            self.state.released = True
            self.state.release_time = GameTime.get_time()
            _log("PEDESTRIAN RELEASED at t={:.3f}".format(self.state.release_time))
        return py_trees.common.Status.SUCCESS


class RecordGhostMetrics(AtomicBehavior):
    """Samples ego / pedestrian state + geometric occlusion every tick."""

    def __init__(self, state, ego, walker, camera, occluder, crossing_loc, name="RecordGhostMetrics"):
        super(RecordGhostMetrics, self).__init__(name)
        self.state = state
        self.ego = ego
        self.walker = walker
        self.camera = camera
        self.occluder = occluder
        self.crossing_loc = crossing_loc

    def _is_occluded(self, wloc):
        """True if the parked occluder's OBB blocks camera->walker line of sight.

        Uses an explicit ray/OBB sampling test instead of World.cast_ray, which
        has an undocumented short range and misses at the 25 m trigger point.
        """
        if self.camera is None or wloc is None or self.occluder is None:
            return None
        try:
            cam = self.camera.get_location()
            target = carla.Location(wloc.x, wloc.y, 1.0)
            tf = self.occluder.get_transform()
            ext = self.occluder.bounding_box.extent
            ctr = self.occluder.bounding_box.location
            yaw = math.radians(tf.rotation.yaw)
            cs, sn = math.cos(yaw), math.sin(yaw)

            def inside(p):
                dx, dy = p.x - tf.location.x, p.y - tf.location.y
                lx = cs * dx + sn * dy
                ly = -sn * dx + cs * dy
                lz = p.z - tf.location.z - ctr.z
                return abs(lx) < ext.x and abs(ly) < ext.y and abs(lz) < ext.z

            for i in range(1, 61):
                f = i / 60.0
                p = carla.Location(
                    cam.x + (target.x - cam.x) * f,
                    cam.y + (target.y - cam.y) * f,
                    cam.z + (target.z - cam.z) * f,
                )
                if inside(p):
                    return True
            return False
        except Exception:
            return None

    def update(self):
        eloc = CarlaDataProvider.get_location(self.ego)
        wloc = CarlaDataProvider.get_location(self.walker) if self.walker else None
        ego_speed = float(CarlaDataProvider.get_velocity(self.ego))
        ctrl = self.ego.get_control()
        t = GameTime.get_time()

        dist = _flat_dist(eloc, wloc) if wloc else None
        cross_dist = _flat_dist(eloc, self.crossing_loc)

        if wloc is not None and dist < self.state.min_dist:
            self.state.min_dist = dist
            self.state.min_dist_time = round(t, 3)

        if self.state.released and self.state.brake_onset is None and ctrl.brake > 0.3:
            self.state.brake_onset = t

        ttc = None
        if self.state.released and cross_dist > 0 and ego_speed > 0.5:
            ttc = round(cross_dist / ego_speed, 3)

        self.state.append_record({
            "t": round(t, 3),
            "ego_x": round(eloc.x, 3), "ego_y": round(eloc.y, 3),
            "ego_speed": round(ego_speed, 3),
            "throttle": round(ctrl.throttle, 3), "brake": round(ctrl.brake, 3),
            "steer": round(ctrl.steer, 3),
            "walker_x": None if wloc is None else round(wloc.x, 3),
            "walker_y": None if wloc is None else round(wloc.y, 3),
            "dist_ego_walker": None if dist is None else round(dist, 3),
            "dist_ego_crossing": round(cross_dist, 3),
            "released": self.state.released,
            "occluded": self._is_occluded(wloc) if not self.state.released else None,
            "ttc": ttc,
        })
        return py_trees.common.Status.RUNNING


class Finalize(AtomicBehavior):
    """Dumps metrics when the scenario behavior completes."""

    def __init__(self, state, sensors=(), name="Finalize"):
        super(Finalize, self).__init__(name)
        self.state = state
        self.sensors = sensors

    def update(self):
        self.state.dump()
        for sensor in self.sensors:
            try:
                sensor.stop()
            except Exception:
                pass
            try:
                sensor.destroy()
            except Exception:
                pass
        return py_trees.common.Status.SUCCESS


class GhostTimeout(AtomicBehavior):
    """Fails safe: succeeds after `duration` seconds from its first tick."""

    def __init__(self, duration, name="GhostTimeout"):
        super(GhostTimeout, self).__init__(name)
        self.duration = float(duration)
        self.start = None
        self.fired = False

    def initialise(self):
        self.start = None
        self.fired = False

    def update(self):
        if self.fired:
            return py_trees.common.Status.SUCCESS
        now = GameTime.get_time()
        if self.start is None:
            self.start = now
        if now - self.start >= self.duration:
            self.fired = True
            _log("TIMEOUT after {:.1f}s".format(self.duration))
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class PedCollisionTrigger(AtomicBehavior):
    """Succeeds as soon as the ego collides with a walker."""

    def __init__(self, state, name="PedCollisionTrigger"):
        super(PedCollisionTrigger, self).__init__(name)
        self.state = state

    def update(self):
        if any("walker" in c["other"] for c in self.state.collisions):
            _log("PEDESTRIAN COLLISION DETECTED at t={:.3f}".format(GameTime.get_time()))
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class GhostPedestrian(BasicScenario):
    """Minimal occluded pedestrian crossing scenario (single ego, Town01)."""

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=120):
        p = config.other_parameters.get("ghost_params", {})

        def gf(key, default):
            return float(p.get(key, default))

        self.trigger_distance = gf("trigger_distance", 25.0)
        self.walker_speed = gf("walker_speed", 1.5)
        self.crossing_ahead = gf("crossing_ahead", 52.0)
        self.occluder_lateral = gf("occluder_lateral", 2.8)
        self.occluder_before = gf("occluder_before", 5.0)
        self.walker_lateral = gf("walker_lateral", 2.5)
        self.walker_end_lateral = gf("walker_end_lateral", -2.0)
        self.end_after = gf("end_after", 20.0)
        self.walker_enabled = int(gf("walker_enabled", 1)) == 1
        self.save_frames = int(gf("save_frames", 1)) == 1
        self.seed = int(gf("seed", 0))
        out_dir = p.get("output_dir") or "/root/autodl-tmp/ghost_runs/{}".format(config.name)
        self.out_dir = out_dir
        self.timeout = timeout

        self.state = _GhostState(out_dir, save_frames=self.save_frames,
                                 frame_every=5, max_frames=200)
        self._trigger_location = config.trigger_points[0].location
        self._wmap = CarlaDataProvider.get_map()
        self._walker = None
        self._camera = None
        self._collision_sensor = None

        _log("init name={} trigger_dist={} speed={} walker_enabled={} out={}".format(
            config.name, self.trigger_distance, self.walker_speed,
            self.walker_enabled, out_dir))

        super(GhostPedestrian, self).__init__(
            "GhostPedestrian", ego_vehicles, config, world, debug_mode,
            criteria_enable=criteria_enable,
        )

    def _initialize_actors(self, config):
        wp = self._wmap.get_waypoint(self._trigger_location)
        fwd = wp.transform.get_forward_vector()
        right = wp.transform.get_right_vector()

        def advance(loc, vec, d):
            return loc + carla.Location(x=vec.x * d, y=vec.y * d)

        self._crossing_loc = advance(self._trigger_location, fwd, self.crossing_ahead)

        # Parked occluder on the right shoulder, slightly before the crossing point.
        occ_loc = advance(self._crossing_loc, right, self.occluder_lateral)
        occ_loc = advance(occ_loc, fwd, -self.occluder_before)
        occ_yaw = math.degrees(math.atan2(fwd.y, fwd.x))
        occ_tf = carla.Transform(
            carla.Location(occ_loc.x, occ_loc.y, 1.0), carla.Rotation(0, occ_yaw, 0),
        )
        bp_lib = CarlaDataProvider.get_world().get_blueprint_library()
        parked_bp = bp_lib.find("vehicle.lincoln.mkz_2020")
        if parked_bp is None:
            parked_bp = bp_lib.filter("vehicle.*")[0]
        parked_bp.set_attribute("role_name", "ghost_occluder")
        occluder = CarlaDataProvider.request_new_actor(parked_bp.id, occ_tf)
        if occluder is None:
            raise ValueError("GhostPedestrian: failed to spawn occluder")
        occluder.set_simulate_physics(False)
        self.other_actors.append(occluder)
        self._occluder = occluder
        _log("occluder at ({:.2f},{:.2f}) yaw={:.1f}".format(occ_loc.x, occ_loc.y, occ_yaw))

        # Pedestrian hidden behind the occluder, at the crossing longitudinal position.
        if self.walker_enabled:
            start = advance(self._crossing_loc, right, self.walker_lateral)
            end = advance(self._crossing_loc, right, self.walker_end_lateral)
            mv = carla.Vector3D(end.x - start.x, end.y - start.y, 0)
            mlen = math.hypot(mv.x, mv.y)
            mv = carla.Vector3D(mv.x / mlen, mv.y / mlen, 0)
            walk_yaw = math.degrees(math.atan2(mv.y, mv.x))
            walk_tf = carla.Transform(
                carla.Location(start.x, start.y, 0.0), carla.Rotation(0, walk_yaw, 0),
            )
            walker = CarlaDataProvider.request_new_actor("walker.pedestrian.0001", walk_tf)
            if walker is None:
                raise ValueError("GhostPedestrian: failed to spawn walker")
            walker.set_simulate_physics(False)
            self.other_actors.append(walker)
            self._walker = walker
            self._walker_start = start
            self._walker_end = end
            self._walker_distance = mlen
            self._walker_duration = mlen / self.walker_speed
            _log("walker ({:.2f},{:.2f})->({:.2f},{:.2f}) yaw={:.1f} dist={:.2f}m dur={:.2f}s".format(
                start.x, start.y, end.x, end.y, walk_yaw, mlen, self._walker_duration))

        # Ego-attached camera + collision sensor. Kept OUT of other_actors so the
        # base-class cleanup does not double-destroy already-closed CARLA sensors.
        world = CarlaDataProvider.get_world()
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "640")
        cam_bp.set_attribute("image_size_y", "360")
        cam_bp.set_attribute("fov", "90")
        cam_tf = carla.Transform(carla.Location(1.6, 0.0, 2.2), carla.Rotation(0, 0, 0))
        self._camera = world.spawn_actor(cam_bp, cam_tf, attach_to=self.ego_vehicles[0])
        self._camera.listen(self.state.camera_cb)

        col_bp = bp_lib.find("sensor.other.collision")
        self._collision_sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=self.ego_vehicles[0])
        self._collision_sensor.listen(self.state.collision_cb)

    def _create_behavior(self):
        root = py_trees.composites.Parallel(
            name="GhostPedestrian", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
        )
        root.add_child(RecordGhostMetrics(
            self.state, self.ego_vehicles[0], self._walker, self._camera,
            getattr(self, "_occluder", None), self._crossing_loc,
        ))

        seq = py_trees.composites.Sequence(name="GhostSequence")
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._crossing_loc, self.trigger_distance,
        ))

        end_dist = self.crossing_ahead + self.end_after
        if self.walker_enabled:
            seq.add_child(ReleasePedestrian(self.state))
            move = py_trees.composites.Parallel(
                name="CrossAndPass", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
            )
            move.add_child(DashWalker(
                self.state, self._walker, self._walker_start, self._walker_end,
                self.walker_speed,
            ))
            # Finish on "ego passed", pedestrian collision, or a hard timeout.
            # The timeout avoids hanging forever when LEAD stops for the ped.
            wait = py_trees.composites.Parallel(
                name="WaitEnd", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            )
            wait.add_child(DriveDistance(self.ego_vehicles[0], end_dist, name="EgoPassed"))
            wait.add_child(PedCollisionTrigger(self.state))
            wait.add_child(GhostTimeout(max(12.0, self._walker_duration + 8.0)))
            move.add_child(wait)
            seq.add_child(move)
            seq.add_child(ActorDestroy(self._walker, name="DestroyWalker"))
        else:
            wait = py_trees.composites.Parallel(
                name="WaitEnd", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            )
            wait.add_child(DriveDistance(self.ego_vehicles[0], end_dist, name="EgoPassed"))
            wait.add_child(GhostTimeout(30.0))
            seq.add_child(wait)

        seq.add_child(Finalize(
            self.state, [self._camera, self._collision_sensor],
        ))
        seq.add_child(WaitForever())
        root.add_child(seq)
        return root

    def _create_test_criteria(self):
        # Route-level criteria (collision etc.) are registered by the leaderboard;
        # pedestrian contact is detected via our collision sensor.
        return []

    def _stop_sensors(self):
        for s in (self._camera, self._collision_sensor):
            if s is None:
                continue
            try:
                s.stop()
            except Exception:
                pass

    def __del__(self):
        try:
            self.state.dump()
        except Exception:
            pass
        try:
            self.remove_all_actors()
        except Exception:
            pass
