#!/usr/bin/env python
"""GhostPedestrianV4 - urban roadside-parking occluded pedestrian scenario.

Differences from v3:
  * Geometry is derived from the map (Town05 road 19 lane -1 by default), not
    hardcoded offsets. The crossing point follows the ego's lane.
  * The occluder is a tall SUV parked in a real Parking lane, placed on the
    road surface using its bounding box (no floating).
  * The pedestrian is a physics-enabled walker driven by carla.WalkerControl,
    so it walks rather than sliding, and its motion is deterministic.
  * Recordings: forward / top-down / chase cameras at full sim rate, with a
    frame manifest so dropped frames can be detected.
  * Per tick we record a *fraction* of the pedestrian body hidden by the
    occluder (not a binary flag), plus the E1-E5 event timeline.

Activated by a route XML like:
  <scenario type="GhostPedestrianV4" name="GP4_22m">
    <trigger_point x="..." y="..." z="0.0" yaw="90"/>
    <ghost_params trigger_distance="22" walker_speed="1.5"
                  crossing_ahead="55" output_dir="/root/autodl-tmp/ghost_runs_v4/gp4_22m"/>
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


GP_TAG = "GP4"
LANE_HALFWIDTH = 1.2   # ego half-width + pedestrian radius
EGO_HALF_LENGTH = 2.45

# LEAD's front camera (sensor rig: rgb_2 at (0.25, 0, 2.25), 384x384, fov 60).
# The occlusion / observability check uses this pose, not the demo camera, so
# the "can the model see it" claim refers to the model's real input.
LEAD_CAM_POS = (0.25, 0.0, 2.25)
LEAD_CAM_FOV = 60.0
# Pedestrian body sampling for the occlusion fraction (15 points).
PED_HALF_WIDTH = 0.16
PED_HEIGHTS = (0.2, 0.6, 1.0, 1.4, 1.75)


def _log(msg):
    print("[{}] {}".format(GP_TAG, msg), flush=True)


def _flat_dist(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _advance(loc, vec, d):
    return carla.Location(x=loc.x + vec.x * d, y=loc.y + vec.y * d, z=loc.z)


class _V4State:
    """Shared state between sensor callbacks and behaviors."""

    def __init__(self, out_dir, trigger_distance=None, save_frames=True,
                 frame_every=1, max_frames=1200, cam_w=1280, cam_h=720):
        self.out_dir = out_dir
        self.trigger_distance = trigger_distance
        self.frames_root = os.path.join(out_dir, "frames")
        os.makedirs(self.frames_root, exist_ok=True)
        self.trace_path = os.path.join(out_dir, "trace.jsonl")
        self.manifest_path = os.path.join(out_dir, "frames_manifest.jsonl")
        with open(self.trace_path, "w"):
            pass
        with open(self.manifest_path, "w"):
            pass
        with open(os.path.join(out_dir, "summary.json"), "w") as f:
            json.dump({"status": "running"}, f)
        self.save_frames = save_frames
        self.frame_every = int(frame_every)
        self.max_frames = int(max_frames)
        self.cam_w = cam_w
        self.cam_h = cam_h
        self.saved = {"lead": 0, "top": 0, "chase": 0}
        self.cam_counter = {"lead": 0, "top": 0, "chase": 0}
        self.records = []
        self.collisions = []
        self.released = False
        self.release_time = None
        self.release_frame = None
        self.actual_trigger_distance_m = None
        self.first_post_release_distance_m = None
        self.release_speed_mps = None
        self.release_ego = None
        self.release_walker = None
        self.walker_speed_mps = None
        self.visible_time = None
        self.lane_entry_time = None
        self.oracle_mode = None
        self.oracle_active_logged = False
        self.oracle_planned_start_time_s = None
        self.oracle_first_active_time_s = None
        self.oracle_first_active_frame = None
        self.oracle_currently_active = False
        self.brake_onset = None
        self.min_dist = float("inf")
        self.min_dist_time = None
        self.stop = False
        self.dumped = False
        self.geom = {}
        self.walker_cmd_applied = 0
        self.last_walker_cmd = None
        self.walker_mode = "unknown"
        self.in_fov_before_release = None

    def camera_cb(self, tag):
        def cb(image):
            if not self.save_frames or self.stop:
                return
            self.cam_counter[tag] += 1
            if self.cam_counter[tag] % self.frame_every != 0:
                return
            if self.saved[tag] >= self.max_frames:
                return
            self.saved[tag] += 1
            d = os.path.join(self.frames_root, tag)
            os.makedirs(d, exist_ok=True)
            image.save_to_disk(os.path.join(d, "%06d.png" % image.frame))
            with open(self.manifest_path, "a") as f:
                f.write(json.dumps({"tag": tag, "frame": int(image.frame),
                                    "t": round(float(image.timestamp), 4)}) + "\n")
        return cb

    def collision_cb(self, event):
        other = event.other_actor.type_id if event.other_actor else "unknown"
        self.collisions.append({"t": round(GameTime.get_time(), 3),
                                "other": other})

    def append_record(self, record):
        if self.dumped:
            return
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
        pre = [r for r in self.records
               if not r["released"] and r.get("occlusion") is not None]
        occ_frac = (round(sum(r["occlusion"] for r in pre) / len(pre), 3)
                    if pre else None)
        fov_frac = (round(sum(r.get("in_fov") or 0.0 for r in pre) / len(pre), 3)
                    if pre else None)
        post = [r for r in self.records if r.get("released")]
        m = {}
        if post and self.release_time is not None:
            t0 = self.release_time
            m["ego_speed_at_release_mps"] = (
                self.release_speed_mps if self.release_speed_mps is not None
                else post[0]["ego_speed"])
            m["available_dist_at_release_m"] = (
                self.actual_trigger_distance_m
                if self.actual_trigger_distance_m is not None
                else post[0]["dist_ego_crossing"])
            vis = next((r for r in post if r.get("occlusion") is not None
                        and r["occlusion"] < 0.5), None)
            m["first_visible_after_release_s"] = (
                round(vis["t"] - t0, 3) if vis else None)
            lane = next((r for r in post if r.get("ped_lateral") is not None
                         and abs(r["ped_lateral"]) < LANE_HALFWIDTH), None)
            m["ped_lane_entry_after_release_s"] = (
                round(lane["t"] - t0, 3) if lane else None)
            cross = next((r for r in post if r.get("ego_front_cross") is not None
                          and r["ego_front_cross"] <= 0), None)
            m["ego_cross_after_release_s"] = (
                round(cross["t"] - t0, 3) if cross else None)
            m["ego_speed_at_cross_mps"] = cross["ego_speed"] if cross else None
            if (m["ego_cross_after_release_s"] is not None
                    and m["ped_lane_entry_after_release_s"] is not None):
                m["reaction_window_s"] = round(
                    m["ego_cross_after_release_s"]
                    - m["ped_lane_entry_after_release_s"], 3)
            sus = None
            for i in range(len(post) - 2):
                if all(post[i + j].get("brake", 0) > 0.3 for j in range(3)):
                    sus = post[i]["t"]
                    break
            m["sustained_brake_after_release_s"] = (
                round(sus - t0, 3) if sus is not None else None)
            decels = [r["decel"] for r in post
                      if r.get("decel") is not None and 0.0 <= r["decel"] <= 12.0]
            m["max_decel_mps2"] = round(max(decels), 2) if decels else None
            v0 = m["ego_speed_at_release_mps"]
            if v0 and m["max_decel_mps2"]:
                req = v0 * v0 / (2.0 * m["max_decel_mps2"])
                m["required_stop_dist_m"] = round(req, 2)
                m["physical_margin_m"] = round(
                    m["available_dist_at_release_m"] - req, 2)
        summary = {
            "status": "final",
            "town": self.geom.get("town"),
            "trigger_distance_m": self.trigger_distance,
            "actual_trigger_distance_m": self.actual_trigger_distance_m,
            "trigger_error_m": (
                round(self.actual_trigger_distance_m - self.trigger_distance, 3)
                if self.actual_trigger_distance_m is not None
                and self.trigger_distance is not None else None),
            "first_post_release_distance_m": self.first_post_release_distance_m,
            "release_speed_mps": self.release_speed_mps,
            "release_frame": self.release_frame,
            "release_ego_x": self.release_ego.x if self.release_ego else None,
            "release_ego_y": self.release_ego.y if self.release_ego else None,
            "release_walker_x": (self.release_walker.x
                                 if self.release_walker else None),
            "release_walker_y": (self.release_walker.y
                                 if self.release_walker else None),
            "walker_speed_mps": self.walker_speed_mps,
            "visible_time_s": (round(self.visible_time - self.release_time, 3)
                               if self.visible_time is not None
                               and self.release_time is not None else None),
            "occlusion_mean_before_release": occ_frac,
            "in_fov_mean_before_release": fov_frac,
            "occluded_samples": len(pre),
            "walker_mode": self.walker_mode,
            "oracle_mode": self.oracle_mode,
            "oracle_planned_start_time_s": (
                round(self.oracle_planned_start_time_s - self.release_time, 3)
                if self.oracle_planned_start_time_s is not None
                and self.release_time is not None else None),
            "oracle_first_active_time_s": (
                round(self.oracle_first_active_time_s - self.release_time, 3)
                if self.oracle_first_active_time_s is not None
                and self.release_time is not None else None),
            "oracle_first_active_frame": self.oracle_first_active_frame,
            "num_records": len(self.records),
            "collisions_all": self.collisions,
            "collision_pedestrian": len(ped_hits) > 0,
            "min_distance_m": (None if self.min_dist == float("inf")
                               else round(self.min_dist, 3)),
            "min_distance_time_s": self.min_dist_time,
            "release_time_s": self.release_time,
            "brake_onset_s": self.brake_onset,
            "brake_onset_after_release_s": (
                round(self.brake_onset - self.release_time, 3)
                if self.brake_onset is not None
                and self.release_time is not None else None),
            "frames_saved": dict(self.saved),
            "geometry": self.geom,
        }
        summary.update(m)
        with open(os.path.join(self.out_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        _log("dumped {} records, frames={}, ped_collision={}, occ={}".format(
            len(self.records), self.saved, summary["collision_pedestrian"],
            occ_frac))


class ReleasePedestrian(AtomicBehavior):
    """Marks release and captures the state at that exact tick."""

    def __init__(self, state, ego, walker, crossing_loc, walker_speed,
                 name="ReleasePedestrian"):
        super(ReleasePedestrian, self).__init__(name)
        self.state = state
        self.ego = ego
        self.walker = walker
        self.crossing_loc = crossing_loc
        self.walker_speed = float(walker_speed)

    def update(self):
        if not self.state.released:
            self.state.released = True
            self.state.release_time = GameTime.get_time()
            self.state.walker_speed_mps = self.walker_speed
            eloc = CarlaDataProvider.get_location(self.ego)
            wloc = (self.walker.get_location()
                    if self.walker else None)
            self.state.actual_trigger_distance_m = round(
                _flat_dist(eloc, self.crossing_loc), 3)
            self.state.release_speed_mps = round(
                float(CarlaDataProvider.get_velocity(self.ego)), 3)
            self.state.release_ego = eloc
            self.state.release_walker = wloc
            try:
                self.state.release_frame = (
                    CarlaDataProvider.get_world().get_snapshot().frame)
            except Exception:
                self.state.release_frame = None
            _log("RELEASED t={:.3f} frame={} dist={:.3f} v={:.3f} "
                 "walker=({:.2f},{:.2f})".format(
                     self.state.release_time, self.state.release_frame,
                     self.state.actual_trigger_distance_m,
                     self.state.release_speed_mps,
                     wloc.x if wloc else -1, wloc.y if wloc else -1))
        return py_trees.common.Status.SUCCESS


class WalkerControlBehavior(AtomicBehavior):
    """Drives the pedestrian with carla.WalkerControl (physics enabled)."""

    def __init__(self, state, walker, start, end, speed, walk_yaw=None,
                 name="WalkerControlBehavior"):
        super(WalkerControlBehavior, self).__init__(name)
        self.state = state
        self.walker = walker
        self.start = start
        self.end = end
        self.speed = float(speed)
        dx, dy = end.x - start.x, end.y - start.y
        n = math.hypot(dx, dy)
        self.direction = carla.Vector3D(dx / n, dy / n, 0.0)
        self.walk_yaw = (walk_yaw if walk_yaw is not None
                         else math.degrees(math.atan2(dy, dx)))
        self.total = n
        self._physics_reasserted = False
        self._probe_t = None
        self._probe_loc = None
        self._fallback = False

    def update(self):
        if not self.state.released:
            return py_trees.common.Status.RUNNING
        if not self._physics_reasserted:
            # The walker is spawned before the first world tick, when its
            # physics body may not exist yet; re-enable it after ticking.
            try:
                self.walker.set_simulate_physics(True)
            except Exception:
                pass
            self._physics_reasserted = True
            _log("walker physics reasserted; loc={}".format(
                self.walker.get_location()))
        now = GameTime.get_time()
        loc = self.walker.get_location()
        if _flat_dist(loc, self.end) < 0.6:
            if not self._fallback and self.state.walker_mode == "unknown":
                self.state.walker_mode = "physics"
            return py_trees.common.Status.SUCCESS
        # Verify that physics actually moves the walker; if not, fall back to
        # a deterministic path so the scenario remains reproducible.
        if not self._fallback:
            if self._probe_t is None:
                self._probe_t = now
                self._probe_loc = loc
            elif now - self._probe_t >= 1.0:
                if _flat_dist(loc, self._probe_loc) < 0.3:
                    self._fallback = True
                    self.state.walker_mode = "fallback"
                    _log("walker physics ineffective (moved {:.2f}m in 1s); "
                         "switching to deterministic path".format(
                             _flat_dist(loc, self._probe_loc)))
        if self._fallback:
            elapsed = max(0.0, now - self.state.release_time)
            frac = (min(1.0, elapsed * self.speed / self.total)
                    if self.total > 0 else 1.0)
            new = carla.Location(
                self.start.x + (self.end.x - self.start.x) * frac,
                self.start.y + (self.end.y - self.start.y) * frac,
                self.start.z + (self.end.z - self.start.z) * frac)
            tf = self.walker.get_transform()
            tf.location = new
            tf.rotation = carla.Rotation(0, self.walk_yaw, 0)
            self.walker.set_transform(tf)
        try:
            self.walker.apply_control(carla.WalkerControl(
                direction=self.direction, speed=self.speed, jump=False))
            self.state.walker_cmd_applied += 1
            self.state.last_walker_cmd = self.speed
        except Exception as exc:
            _log("walker control failed: {}".format(exc))
        return py_trees.common.Status.RUNNING


class RecordMetrics(AtomicBehavior):
    """Per-tick telemetry, including pedestrian body occlusion fraction."""

    def __init__(self, state, ego, walker, camera, occluder, crossing_loc,
                 right=None, name="RecordMetrics"):
        super(RecordMetrics, self).__init__(name)
        self.state = state
        self.ego = ego
        self.walker = walker
        self.camera = camera
        self.occluder = occluder
        self.crossing_loc = crossing_loc
        self.right = right if right is not None else carla.Vector3D(1, 0, 0)
        self._prev_speed = None
        self._prev_t = None

    def _visibility(self, wloc):
        """Fraction of the pedestrian body hidden from LEAD's front camera.

        Returns (occlusion, in_fov): occlusion is 1 - (body points that are
        both inside the 60 deg front-camera frustum and not blocked by the
        parked occluder). Points outside the frustum count as not visible,
        which is what "the model cannot see it" means.
        """
        if self.ego is None or wloc is None:
            return None, None
        try:
            ego_tf = self.ego.get_transform()
            cam = ego_tf.transform(carla.Location(*LEAD_CAM_POS))
            fwd = ego_tf.get_forward_vector()
            right = ego_tf.get_right_vector()
            up = ego_tf.get_up_vector()
            feet_z = wloc.z
            if self.walker is not None:
                bb = self.walker.bounding_box
                feet_z = wloc.z + bb.location.z - bb.extent.z
            occ_box = None
            if self.occluder is not None:
                occ_box = (self.occluder.get_transform(),
                           self.occluder.bounding_box)
            half = math.tan(math.radians(LEAD_CAM_FOV) / 2.0)
            total = 0
            visible = 0
            infov = 0
            for dl in (-PED_HALF_WIDTH, 0.0, PED_HALF_WIDTH):
                for h in PED_HEIGHTS:
                    total += 1
                    px = wloc.x + right.x * dl
                    py = wloc.y + right.y * dl
                    pz = feet_z + h
                    vx, vy, vz = px - cam.x, py - cam.y, pz - cam.z
                    depth = vx * fwd.x + vy * fwd.y + vz * fwd.z
                    if depth <= 1e-3:
                        continue
                    lat = vx * right.x + vy * right.y + vz * right.z
                    vert = vx * up.x + vy * up.y + vz * up.z
                    if abs(lat) > half * depth or abs(vert) > half * depth:
                        continue
                    infov += 1
                    if occ_box is not None and self._ray_hits_obb(
                            cam, carla.Location(px, py, pz), occ_box[0],
                            occ_box[1]):
                        continue
                    visible += 1
            occ = 1.0 - visible / float(max(1, total))
            return round(occ, 3), round(infov / float(max(1, total)), 3)
        except Exception:
            return None, None

    @staticmethod
    def _ray_hits_obb(a, b, otf, bb):
        yaw = math.radians(otf.rotation.yaw)
        cs, sn = math.cos(yaw), math.sin(yaw)
        ext = bb.extent
        ctr_z = bb.location.z
        for i in range(1, 50):
            f = i / 50.0
            x = a.x + (b.x - a.x) * f - otf.location.x
            y = a.y + (b.y - a.y) * f - otf.location.y
            z = a.z + (b.z - a.z) * f - otf.location.z - ctr_z
            lx = cs * x + sn * y
            ly = -sn * x + cs * y
            if abs(lx) <= ext.x and abs(ly) <= ext.y and abs(z) <= ext.z:
                return True
        return False

    def update(self):
        if self.state.dumped:
            return py_trees.common.Status.SUCCESS
        eloc = CarlaDataProvider.get_location(self.ego)
        wloc = self.walker.get_location() if self.walker else None
        ego_speed = float(CarlaDataProvider.get_velocity(self.ego))
        ctrl = self.ego.get_control()
        t = GameTime.get_time()
        dist = _flat_dist(eloc, wloc) if wloc else None
        cross_dist = _flat_dist(eloc, self.crossing_loc)
        if (self.state.released
                and self.state.first_post_release_distance_m is None):
            self.state.first_post_release_distance_m = round(cross_dist, 3)
        if wloc is not None and dist < self.state.min_dist:
            self.state.min_dist = dist
            self.state.min_dist_time = round(t, 3)
        if (self.state.released and self.state.brake_onset is None
                and ctrl.brake > 0.3):
            self.state.brake_onset = t
        decel = None
        if (self._prev_speed is not None and self._prev_t is not None
                and t > self._prev_t):
            decel = round((self._prev_speed - ego_speed) / (t - self._prev_t), 3)
        self._prev_speed = ego_speed
        self._prev_t = t
        occ, infov = self._visibility(wloc)
        ped_lateral = None if wloc is None else round(
            (wloc.x - self.crossing_loc.x) * self.right.x
            + (wloc.y - self.crossing_loc.y) * self.right.y, 3)
        ego_front_cross = round(cross_dist - EGO_HALF_LENGTH, 3)
        if (self.state.released and self.state.visible_time is None
                and wloc is not None and occ is not None and occ < 0.5):
            self.state.visible_time = t
            _log("VISIBLE (occ<0.5) t={:.3f} occ={:.2f} fov={:.2f}".format(
                t, occ, infov or 0.0))
        if (self.state.released and self.state.lane_entry_time is None
                and ped_lateral is not None
                and abs(ped_lateral) < LANE_HALFWIDTH):
            self.state.lane_entry_time = t
        try:
            rec_frame = CarlaDataProvider.get_world().get_snapshot().frame
        except Exception:
            rec_frame = None
        self.state.append_record({
            "t": round(t, 3), "frame": rec_frame,
            "ego_x": round(eloc.x, 3), "ego_y": round(eloc.y, 3),
            "ego_speed": round(ego_speed, 3),
            "throttle": round(ctrl.throttle, 3), "brake": round(ctrl.brake, 3),
            "steer": round(ctrl.steer, 3),
            "walker_x": None if wloc is None else round(wloc.x, 3),
            "walker_y": None if wloc is None else round(wloc.y, 3),
            "dist_ego_walker": None if dist is None else round(dist, 3),
            "dist_ego_crossing": round(cross_dist, 3),
            "released": self.state.released,
            "oracle_active": self.state.oracle_currently_active,
            "occlusion": occ,
            "in_fov": infov,
            "decel": decel,
            "ped_lateral": ped_lateral,
            "ego_front_cross": ego_front_cross,
            "walker_cmd": self.state.last_walker_cmd,
            "walker_cmd_n": self.state.walker_cmd_applied,
        })
        return py_trees.common.Status.RUNNING


class OracleBrake(AtomicBehavior):
    """Counterfactual full brake, applied before RecordMetrics ticks."""

    def __init__(self, state, ego, mode, brake=1.0, delay=0.0,
                 name="OracleBrake"):
        super(OracleBrake, self).__init__(name)
        self.state = state
        self.ego = ego
        self.mode = mode
        self.brake_cmd = float(brake)
        self.delay = float(delay)

    def _resolve_start(self):
        if self.mode == "immediate":
            return self.state.release_time
        if self.mode in ("visible", "visible_delay"):
            if self.state.visible_time is None:
                return None
            extra = self.delay if self.mode == "visible_delay" else 0.0
            return self.state.visible_time + extra
        return None

    def update(self):
        if not self.state.released:
            return py_trees.common.Status.RUNNING
        start = self._resolve_start()
        if start is None:
            return py_trees.common.Status.RUNNING
        now = GameTime.get_time()
        if now < start:
            self.state.oracle_currently_active = False
            return py_trees.common.Status.RUNNING
        try:
            ctrl = self.ego.get_control()
            ctrl.throttle = 0.0
            ctrl.brake = self.brake_cmd
            self.ego.apply_control(ctrl)
            self.state.oracle_currently_active = True
            if not self.state.oracle_active_logged:
                self.state.oracle_active_logged = True
                self.state.oracle_planned_start_time_s = start
                self.state.oracle_first_active_time_s = now
                try:
                    self.state.oracle_first_active_frame = (
                        CarlaDataProvider.get_world().get_snapshot().frame)
                except Exception:
                    self.state.oracle_first_active_frame = None
                _log("ORACLE active t={:.3f} mode={} brake={:.2f} lag={:.3f}".format(
                    now, self.mode, self.brake_cmd, now - start))
        except Exception as exc:
            self.state.oracle_currently_active = False
            _log("ORACLE apply failed: {}".format(exc))
        return py_trees.common.Status.RUNNING


class Finalize(AtomicBehavior):
    def __init__(self, state, sensors=(), name="Finalize"):
        super(Finalize, self).__init__(name)
        self.state = state
        self.sensors = sensors

    def update(self):
        self.state.dump()
        for s in self.sensors:
            try:
                s.stop()
            except Exception:
                pass
            try:
                s.destroy()
            except Exception:
                pass
        return py_trees.common.Status.SUCCESS


class GhostTimeout(AtomicBehavior):
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
    def __init__(self, state, name="PedCollisionTrigger"):
        super(PedCollisionTrigger, self).__init__(name)
        self.state = state

    def update(self):
        if any("walker" in c["other"] for c in self.state.collisions):
            _log("PED COLLISION t={:.3f}".format(GameTime.get_time()))
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class GhostPedestrianV4(BasicScenario):
    """Urban roadside-parking ghost pedestrian (map-derived, physics walker)."""

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=180):
        p = config.other_parameters.get("ghost_params", {})

        def gf(key, default):
            return float(p.get(key, default))

        self.trigger_distance = gf("trigger_distance", 22.0)
        self.walker_speed = gf("walker_speed", 1.5)
        self.crossing_ahead = gf("crossing_ahead", 55.0)
        self.occluder_before = gf("occluder_before", 2.0)
        self.occluder_nose = gf("occluder_nose", 1.2)
        self.walker_start_off = gf("walker_start_off", 4.0)
        self.walker_end_off = gf("walker_end_off", -5.0)
        self.end_after = gf("end_after", 18.0)
        self.walker_enabled = int(gf("walker_enabled", 1)) == 1
        self.occluder_enabled = int(gf("occluder_enabled", 1)) == 1
        self.save_frames = int(gf("save_frames", 1)) == 1
        self.frame_every = int(gf("frame_every", 1))
        self.max_frames = int(gf("max_frames", 1200))
        self.cam_w = int(gf("cam_w", 1280))
        self.cam_h = int(gf("cam_h", 720))
        self.save_top = int(gf("save_top", 1)) == 1
        self.save_chase = int(gf("save_chase", 1)) == 1
        self.seed = int(gf("seed", 0))
        self.oracle_mode = (p.get("oracle_mode", "") or "").strip()
        self.oracle_brake = gf("oracle_brake", 1.0)
        self.oracle_delay = gf("oracle_delay", 0.0)
        out_dir = p.get("output_dir") or \
            "/root/autodl-tmp/ghost_runs_v4/{}".format(config.name)
        self.out_dir = out_dir

        self.state = _V4State(out_dir, trigger_distance=self.trigger_distance,
                              save_frames=self.save_frames,
                              frame_every=self.frame_every,
                              max_frames=self.max_frames,
                              cam_w=self.cam_w, cam_h=self.cam_h)
        self.state.oracle_mode = self.oracle_mode or None
        self._trigger_location = config.trigger_points[0].location
        self._wmap = CarlaDataProvider.get_map()
        self._walker = None
        self._occluder = None
        self._cameras = []
        self._camera = None
        self._camera_top = None
        self._camera_chase = None
        self._collision_sensor = None
        self._crossing_right = carla.Vector3D(1, 0, 0)
        self._walker_walk_yaw = 0.0
        _log("init name={} trigger={} speed={} out={}".format(
            config.name, self.trigger_distance, self.walker_speed, out_dir))

        super(GhostPedestrianV4, self).__init__(
            "GhostPedestrianV4", ego_vehicles, config, world, debug_mode,
            criteria_enable=criteria_enable)

    # ------------------------------------------------------------------
    def _crossing_waypoint(self):
        wp0 = self._wmap.get_waypoint(
            self._trigger_location, project_to_road=True,
            lane_type=carla.LaneType.Driving)
        nxt = wp0.next(self.crossing_ahead)
        return nxt[0] if nxt else wp0

    def _find_parking_waypoint(self, cross_wp):
        before = cross_wp.previous(self.occluder_before)
        base = before[0] if before else cross_wp
        park = base.get_right_lane()
        while park is not None and park.lane_type not in (
                carla.LaneType.Parking, carla.LaneType.Shoulder):
            park = park.get_right_lane()
        return park if park is not None else base.get_right_lane()

    def _initialize_actors(self, config):
        try:
            town = CarlaDataProvider.get_world().get_map().name.split("/")[-1]
        except Exception:
            town = None
        cross_wp = self._crossing_waypoint()
        cross_tf = cross_wp.transform
        self._crossing_loc = cross_tf.location
        right = cross_tf.get_right_vector()
        self._crossing_right = right
        bp_lib = CarlaDataProvider.get_world().get_blueprint_library()

        self.state.geom = {
            "town": town,
            "crossing": [round(cross_tf.location.x, 2),
                         round(cross_tf.location.y, 2)],
            "crossing_yaw": round(cross_tf.rotation.yaw, 1),
            "crossing_road_id": cross_wp.road_id,
            "crossing_lane_id": cross_wp.lane_id,
            "crossing_ahead_m": self.crossing_ahead,
            "walker_start_off_m": self.walker_start_off,
            "walker_end_off_m": self.walker_end_off,
            "occluder_before_m": self.occluder_before,
        }

        # Parked SUV in the real parking lane, on the road surface. Its nose is
        # placed `occluder_nose` metres before the crossing line, and the
        # pedestrian waits just past that nose on the walkable strip, so the
        # sight line from LEAD's front camera passes through the vehicle.
        if self.occluder_enabled:
            park_wp = self._find_parking_waypoint(cross_wp)
            if park_wp is None:
                raise ValueError("GhostPedestrianV4: no parking/shoulder lane")
            pt = park_wp.transform
            occ_bp = bp_lib.find("vehicle.nissan.patrol")
            if occ_bp is None:
                occ_bp = bp_lib.filter("vehicle.*suv*")[0]
            occ_bp.set_attribute("role_name", "gp4_occluder")
            occluder = CarlaDataProvider.request_new_actor(
                occ_bp.id, carla.Transform(
                    carla.Location(pt.location.x, pt.location.y,
                                   pt.location.z),
                    carla.Rotation(0, pt.rotation.yaw, 0)))
            if occluder is None:
                raise ValueError("GhostPedestrianV4: occluder spawn failed")
            occluder.set_simulate_physics(False)
            # Re-place on the road surface with the nose rule (map-derived
            # lateral offset, vehicle-derived longitudinal offset).
            right_c = cross_tf.get_right_vector()
            fwd_c = cross_tf.get_forward_vector()
            lat_off = ((pt.location.x - cross_tf.location.x) * right_c.x
                       + (pt.location.y - cross_tf.location.y) * right_c.y)
            bb = occluder.bounding_box
            back = bb.extent.x + self.occluder_nose
            occ_x = cross_tf.location.x + right_c.x * lat_off - fwd_c.x * back
            occ_y = cross_tf.location.y + right_c.y * lat_off - fwd_c.y * back
            occluder.set_transform(carla.Transform(
                carla.Location(occ_x, occ_y, pt.location.z),
                carla.Rotation(0, cross_tf.rotation.yaw, 0)))
            self.other_actors.append(occluder)
            self._occluder = occluder
            self.state.geom["occluder"] = {
                "x": round(occ_x, 2), "y": round(occ_y, 2),
                "yaw": round(cross_tf.rotation.yaw, 1),
                "lane_type": str(park_wp.lane_type),
                "blueprint": occ_bp.id,
                "length_m": round(bb.extent.x * 2, 2),
                "width_m": round(bb.extent.y * 2, 2),
                "height_m": round(bb.extent.z * 2, 2),
                "bbox_bottom_z": round(pt.location.z + bb.location.z
                                       - bb.extent.z, 3),
                "road_z": round(pt.location.z, 3),
                "nose_gap_m": self.occluder_nose,
            }
            _log("occluder {} at ({:.2f},{:.2f}) yaw={:.1f} lane={} "
                 "nose_gap={:.2f}".format(
                     occ_bp.id, occ_x, occ_y, cross_tf.rotation.yaw,
                     park_wp.lane_type, self.occluder_nose))

        # Pedestrian: find a free spawn behind the occluder on the far side.
        world = CarlaDataProvider.get_world()
        if self.walker_enabled:
            walker_bp = bp_lib.filter("walker.pedestrian.*")[0]
            walker_bp.set_attribute("role_name", "gp4_walker")
            ground = cross_tf.location.z
            chosen = None
            for s_off in (0.0, 1.0, -1.0, 2.0, -2.0):
                base_wp = self._wmap.get_waypoint(
                    _advance(cross_tf.location, cross_tf.get_forward_vector(),
                             s_off), project_to_road=True,
                    lane_type=carla.LaneType.Driving)
                b = base_wp.transform
                r = b.get_right_vector()
                walk_yaw = math.degrees(math.atan2(-r.y, -r.x))
                for off in (self.walker_start_off, self.walker_start_off - 0.3,
                            self.walker_start_off + 0.3,
                            self.walker_start_off - 0.6):
                    start = carla.Location(
                        b.location.x + r.x * off, b.location.y + r.y * off,
                        ground + 0.94)
                    try:
                        walker = world.spawn_actor(
                            walker_bp, carla.Transform(
                                start, carla.Rotation(0, walk_yaw, 0)))
                    except Exception:
                        walker = None
                    if walker is not None:
                        chosen = (walker, start, b, r, off, s_off, walk_yaw)
                        break
                if chosen:
                    break
            if not chosen:
                raise ValueError("GhostPedestrianV4: walker spawn failed")
            walker, start, b, r, off, s_off, walk_yaw = chosen
            try:
                walker.set_simulate_physics(True)
            except Exception:
                pass
            # The actor transform is a placeholder until the next world tick,
            # so derive the walk line from the requested spawn location.
            end = carla.Location(
                b.location.x + r.x * self.walker_end_off,
                b.location.y + r.y * self.walker_end_off,
                start.z)
            self.other_actors.append(walker)
            self._walker = walker
            self._walker_start = start
            self._walker_end = end
            self._walker_walk_yaw = walk_yaw
            self.state.geom["walker"] = {
                "start": [round(start.x, 2), round(start.y, 2)],
                "end": [round(end.x, 2), round(end.y, 2)],
                "start_off_m": off, "s_offset_m": s_off,
                "walk_yaw": round(walk_yaw, 1),
                "blueprint": walker_bp.id,
                "walk_distance_m": round(_flat_dist(start, end), 2),
            }
            _log("walker at ({:.2f},{:.2f})->({:.2f},{:.2f}) off={:.1f} "
                 "dist={:.2f}m".format(
                     start.x, start.y, end.x, end.y, off,
                     _flat_dist(start, end)))

        # Cameras + collision sensor (kept out of other_actors).
        # The primary view is LEAD's *actual* front camera pose (rgb_2:
        # (0.25, 0, 2.25), 384x384, fov 60), so the video shows what the model
        # really gets, not a proxy. top/chase are extra observation views.
        # Experiments (save_frames=0) spawn no extra camera at all: rendering
        # them costs VRAM and destabilises the headless renderer.
        world = CarlaDataProvider.get_world()
        ego = self.ego_vehicles[0]
        if self.save_frames:
            lead_bp = bp_lib.find("sensor.camera.rgb")
            lead_bp.set_attribute("image_size_x", "384")
            lead_bp.set_attribute("image_size_y", "384")
            lead_bp.set_attribute("fov", str(int(LEAD_CAM_FOV)))
            lead_bp.set_attribute("sensor_tick", "0.0")
            lead_tf = carla.Transform(
                carla.Location(*LEAD_CAM_POS), carla.Rotation(0, 0, 0))
            self._camera = world.spawn_actor(lead_bp, lead_tf, attach_to=ego)
            self._camera.listen(self.state.camera_cb("lead"))
            self._cameras.append(self._camera)
            if self.save_top:
                top_bp = bp_lib.find("sensor.camera.rgb")
                top_bp.set_attribute("image_size_x", "800")
                top_bp.set_attribute("image_size_y", "450")
                top_bp.set_attribute("fov", "100")
                top_bp.set_attribute("sensor_tick", "0.0")
                top_tf = carla.Transform(carla.Location(-6.0, 0.0, 18.0),
                                         carla.Rotation(-70, 0, 0))
                self._camera_top = world.spawn_actor(top_bp, top_tf,
                                                     attach_to=ego)
                self._camera_top.listen(self.state.camera_cb("top"))
                self._cameras.append(self._camera_top)
            if self.save_chase:
                chase_bp = bp_lib.find("sensor.camera.rgb")
                chase_bp.set_attribute("image_size_x", "960")
                chase_bp.set_attribute("image_size_y", "540")
                chase_bp.set_attribute("fov", "80")
                chase_bp.set_attribute("sensor_tick", "0.0")
                chase_tf = carla.Transform(carla.Location(-8.0, 0.0, 4.0),
                                           carla.Rotation(-12, 0, 0))
                self._camera_chase = world.spawn_actor(chase_bp, chase_tf,
                                                       attach_to=ego)
                self._camera_chase.listen(self.state.camera_cb("chase"))
                self._cameras.append(self._camera_chase)
        col_bp = bp_lib.find("sensor.other.collision")
        self._collision_sensor = world.spawn_actor(
            col_bp, carla.Transform(), attach_to=ego)
        self._collision_sensor.listen(self.state.collision_cb)

    def _create_behavior(self):
        root = py_trees.composites.Parallel(
            name="GhostPedestrianV4",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # Backstop: never let the scenario tick forever (e.g. an Oracle stop
        # leaves the ego short of the route end).
        root.add_child(GhostTimeout(45.0, name="ScenarioGuardTimeout"))
        if self.oracle_mode:
            root.add_child(OracleBrake(
                self.state, self.ego_vehicles[0], self.oracle_mode,
                brake=self.oracle_brake, delay=self.oracle_delay))
        root.add_child(RecordMetrics(
            self.state, self.ego_vehicles[0], self._walker, self._camera,
            self._occluder, self._crossing_loc, self._crossing_right))
        seq = py_trees.composites.Sequence(name="GhostSequence")
        seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._crossing_loc, self.trigger_distance))
        end_dist = self.crossing_ahead + self.end_after
        if self.walker_enabled:
            seq.add_child(ReleasePedestrian(
                self.state, self.ego_vehicles[0], self._walker,
                self._crossing_loc, self.walker_speed))
            move = py_trees.composites.Parallel(
                name="CrossAndPass",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            move.add_child(WalkerControlBehavior(
                self.state, self._walker, self._walker_start,
                self._walker_end, self.walker_speed,
                walk_yaw=self._walker_walk_yaw))
            wait = py_trees.composites.Parallel(
                name="WaitEnd",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            wait.add_child(DriveDistance(self.ego_vehicles[0], end_dist,
                                         name="EgoPassed"))
            wait.add_child(PedCollisionTrigger(self.state))
            wait.add_child(GhostTimeout(max(14.0, 10.0 + 0.5)))
            move.add_child(wait)
            seq.add_child(move)
            seq.add_child(ActorDestroy(self._walker, name="DestroyWalker"))
        else:
            wait = py_trees.composites.Parallel(
                name="WaitEnd",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            wait.add_child(DriveDistance(self.ego_vehicles[0], end_dist,
                                         name="EgoPassed"))
            wait.add_child(GhostTimeout(30.0))
            seq.add_child(wait)
        seq.add_child(Finalize(self.state,
                               [self._camera, self._collision_sensor]
                               + self._cameras[1:]))
        seq.add_child(WaitForever())
        root.add_child(seq)
        return root

    def _create_test_criteria(self):
        return []

    def _stop_sensors(self):
        for s in self._cameras + [self._collision_sensor]:
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
