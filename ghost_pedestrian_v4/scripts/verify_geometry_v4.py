#!/usr/bin/env python3
"""Geometry self-check for GhostPedestrianV4 (no LEAD needed).

Connects to a running CARLA on Town05_Opt, reproduces the scenario's placement
rules and prints:

  * the lane cross-section at the crossing (Driving / Parking / Shoulder /
    Sidewalk),
  * the parked SUV's bounding-box bottom vs the road surface (no floating),
  * the pedestrian body occlusion/visibility fraction seen from LEAD's front
    camera for a grid of lateral offsets and ego distances.

Usage:  python scripts/verify_geometry_v4.py [port] > results/geometry_check.txt
"""
import math
import sys

import carla

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
TRIGGER = carla.Location(x=150.6, y=-97.8, z=0.0)
CROSSING_AHEAD = 55.0
NOSE_GAP = 1.2
LEAD_CAM_POS = (0.25, 0.0, 2.25)
LEAD_CAM_FOV = 60.0
PED_HALF_WIDTH = 0.16
PED_HEIGHTS = (0.2, 0.6, 1.0, 1.4, 1.75)


def inside(p, loc, yaw_deg, ext, ctr_z):
    yaw = math.radians(yaw_deg)
    cs, sn = math.cos(yaw), math.sin(yaw)
    dx, dy, dz = p[0] - loc.x, p[1] - loc.y, p[2] - loc.z - ctr_z
    lx = cs * dx + sn * dy
    ly = -sn * dx + cs * dy
    return abs(lx) <= ext.x and abs(ly) <= ext.y and abs(dz) <= ext.z


def visibility(dist, lateral, cross, right, occ, ground_z):
    ex = cross.location.x - 0.0 + right.x * 0.0
    ey = cross.location.y - dist
    cam = (ex, ey, ground_z + LEAD_CAM_POS[2])
    wx = cross.location.x + right.x * lateral
    wy = cross.location.y + right.y * lateral
    fwd = (0.0, 1.0)
    half = math.tan(math.radians(LEAD_CAM_FOV) / 2.0)
    vis = 0
    fov = 0
    for dl in (-PED_HALF_WIDTH, 0.0, PED_HALF_WIDTH):
        for h in PED_HEIGHTS:
            px = wx + right.x * dl
            py = wy + right.y * dl
            pz = ground_z + h
            depth = (py - cam[1]) * fwd[1]
            lat = px - cam[0]
            vert = pz - cam[2]
            if depth <= 0 or abs(lat) > half * depth or abs(vert) > half * depth:
                continue
            fov += 1
            blocked = False
            for i in range(1, 50):
                f = i / 50.0
                q = (cam[0] + (px - cam[0]) * f,
                     cam[1] + (py - cam[1]) * f,
                     cam[2] + (pz - cam[2]) * f)
                if inside(q, occ["loc"], occ["yaw"], occ["ext"], occ["ctr_z"]):
                    blocked = True
                    break
            if not blocked:
                vis += 1
    return vis / 15.0, fov / 15.0


def main():
    client = carla.Client("127.0.0.1", PORT)
    client.set_timeout(10.0)
    world = client.get_world()
    wmap = world.get_map()
    lib = world.get_blueprint_library()
    print("map:", wmap.name)

    wp0 = wmap.get_waypoint(TRIGGER, project_to_road=True,
                            lane_type=carla.LaneType.Driving)
    cross_wp = (wp0.next(CROSSING_AHEAD) or [wp0])[0]
    cross = cross_wp.transform
    right = cross.get_right_vector()
    fwd = cross.get_forward_vector()
    ground_z = cross.location.z
    print("crossing: (%.2f, %.2f) z=%.3f yaw=%.1f road=%d lane=%d"
          % (cross.location.x, cross.location.y, ground_z,
             cross.rotation.yaw, cross_wp.road_id, cross_wp.lane_id))

    print("\ncross-section (lateral offset -> lane type):")
    lat = 0.0
    wp = cross_wp
    for _ in range(5):
        wp = wp.get_right_lane()
        if wp is None:
            break
        lat += wp.lane_width
        print("  +%4.2f m  lane %+d  %-9s width %.2f"
              % (lat, wp.lane_id, str(wp.lane_type).split(".")[-1],
                 wp.lane_width))

    # parked SUV with the same nose rule as the scenario
    park = None
    wp = cross_wp.previous(4.0)
    base = wp[0] if wp else cross_wp
    park = base.get_right_lane()
    while park is not None and park.lane_type not in (
            carla.LaneType.Parking, carla.LaneType.Shoulder):
        park = park.get_right_lane()
    pt = park.transform
    lat_off = ((pt.location.x - cross.location.x) * right.x
               + (pt.location.y - cross.location.y) * right.y)
    bp = lib.find("vehicle.nissan.patrol")
    bb0 = None
    occ = None
    back = 0.0
    # spawn once to read the bbox, then re-place exactly like the scenario
    veh = world.try_spawn_actor(bp, carla.Transform(
        carla.Location(pt.location.x, pt.location.y, pt.location.z),
        carla.Rotation(0, cross.rotation.yaw, 0)))
    if veh is None:
        print("occluder spawn failed")
        return
    veh.set_simulate_physics(False)
    bb = veh.bounding_box
    back = bb.extent.x + NOSE_GAP
    ox = cross.location.x + right.x * lat_off - fwd.x * back
    oy = cross.location.y + right.y * lat_off - fwd.y * back
    veh.set_transform(carla.Transform(
        carla.Location(ox, oy, pt.location.z),
        carla.Rotation(0, cross.rotation.yaw, 0)))
    bb = veh.bounding_box
    print("\noccluder: %s  L=%.2f W=%.2f H=%.2f"
          % (bp.id, bb.extent.x * 2, bb.extent.y * 2, bb.extent.z * 2))
    print("  center (%.2f, %.2f) lateral %.2f m, nose %.2f m before crossing"
          % (ox, oy, lat_off, NOSE_GAP))
    print("  bbox bottom z=%.3f  road z=%.3f  delta=%+.3f m (0 = on the road)"
          % (pt.location.z + bb.location.z - bb.extent.z, ground_z,
             pt.location.z + bb.location.z - bb.extent.z - ground_z))
    occ = {"loc": veh.get_transform().location,
           "yaw": veh.get_transform().rotation.yaw,
           "ext": bb.extent, "ctr_z": bb.location.z}

    print("\npedestrian visibility from LEAD front camera "
          "(1.00 = fully visible):")
    print("  lateral  d=8    d=12   d=16   d=20   d=24   d=30   d=40")
    for lateral in (2.75, 3.2, 3.6, 4.0, 4.36, 4.5, 5.4):
        cells = ["%.2f" % visibility(d, lateral, cross, right, occ, ground_z)[0]
                 for d in (8, 12, 16, 20, 24, 30, 40)]
        print("  %5.2f    %s" % (lateral, "  ".join(cells)))
    print("\n(4.36-6.39 m is the Sidewalk; 1.75-3.75 m is the Parking lane.)")
    veh.destroy()


if __name__ == "__main__":
    main()
