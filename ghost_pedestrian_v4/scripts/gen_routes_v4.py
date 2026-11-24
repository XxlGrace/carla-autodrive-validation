#!/usr/bin/env python3
"""Generate GhostPedestrianV4 route XMLs (Town05_Opt, road 19 lane -1).

Geometry is derived by the scenario from the map: the crossing point follows
the ego lane `crossing_ahead` metres ahead of the trigger, the SUV is parked in
the real Parking lane with its nose `occluder_nose` metres before the crossing,
and the pedestrian starts just past the nose on the walkable strip and walks
across at `walker_speed`.

The route list for the runner is "<file.xml> [pred]"; "pred" enables the
LEAD-side GP_PRED_LOG hook so model perception can be aligned to the trace.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTES = os.path.join(HERE, "..", "routes")
os.makedirs(ROUTES, exist_ok=True)

WEATHER = '''    <weathers>
      <weather route_percentage="0" cloudiness="10.0" precipitation="0.0"
               precipitation_deposits="0.0" wetness="0.0" wind_intensity="0.0"
               sun_azimuth_angle="180.0" sun_altitude_angle="45.0"
               fog_density="0.0" fog_distance="0.0" />
    </weathers>'''

WAYPOINTS = [
    (150.6, -97.8),   # s~130 (trigger)
    (151.4, -78.2),   # s~150
    (151.5, -58.2),   # s~170
    (151.5, -38.2),   # s~190
    (151.5, -14.2),   # s~214
]
WAYPOINTS_XML = "    <waypoints>\n" + "\n".join(
    '      <position x="{}" y="{}" z="0.0" />'.format(x, y)
    for x, y in WAYPOINTS) + "\n    </waypoints>"
TRIGGER_XML = '        <trigger_point x="150.6" y="-97.8" z="0.0" yaw="80" />'

COMMON = dict(
    walker_speed="1.5", crossing_ahead="55", occluder_nose="1.2",
    walker_start_off="4.0", walker_end_off="-5.0", end_after="18", seed="0",
)
EXP = dict(COMMON, save_frames="0")
DEMO = dict(COMMON, save_frames="1", frame_every="1", max_frames="800",
            save_top="1", save_chase="1")
OUTROOT = "/root/autodl-tmp/ghost_runs_v4"


def scenario_block(name, params):
    attrs = " ".join('{}="{}"'.format(k, v) for k, v in params.items())
    return ('      <scenario type="GhostPedestrianV4" name="{name}">\n'
            '{trigger}\n        <ghost_params {attrs} />\n'
            '      </scenario>').format(name=name, trigger=TRIGGER_XML, attrs=attrs)


def write_route(fname, scenarios):
    body = "\n".join(scenarios) if scenarios else ""
    xml = ('<?xml version="1.0" encoding="UTF-8"?>\n<routes>\n'
           '  <route id="0" town="Town05_Opt">\n{weather}\n{waypoints}\n'
           '    <scenarios>\n{scenarios}\n    </scenarios>\n'
           '  </route>\n</routes>\n').format(
               weather=WEATHER, waypoints=WAYPOINTS_XML, scenarios=body)
    with open(os.path.join(ROUTES, fname), "w") as f:
        f.write(xml)


def run(name, params, base=EXP):
    p = dict(base)
    p.update(params)
    p["output_dir"] = OUTROOT + "/" + name
    write_route(name + ".xml", [scenario_block("GP4_" + name, p)])


# Controls
write_route("v4_baseline.xml", [])
run("v4_occluder_only", dict(walker_enabled="0"))
run("v4_no_occluder", dict(occluder_enabled="0", trigger_distance="20"))

# LEAD sweep (no frames)
for d in (8, 12, 16, 20, 24, 26, 28, 30, 34, 40):
    run("v4_d{}m".format(d), dict(trigger_distance=str(d)))

# Oracle counterfactuals at the collision distance (20 m in the v4 geometry)
for mode in ("immediate", "visible"):
    run("v4_d20m_or_{}".format(mode),
        dict(trigger_distance="20", oracle_mode=mode, oracle_brake="1.0",
             oracle_delay="0.0"))

# Boundary repeats (20 m / 24 m / 26 m)
for d in (20, 24, 26):
    for r in range(1, 4):
        run("v4_boundary{}_r{}".format(d, r), dict(trigger_distance=str(d)))

# Walker speed at the collision distance
for sp in ("1.0", "2.0"):
    run("v4_s{}_d20m".format(sp.replace(".", "p")),
        dict(walker_speed=sp, trigger_distance="20"))

# Demo variants (3 cameras + LEAD input view)
run("v4_demo_occluder_only", dict(walker_enabled="0", trigger_distance="20"),
    base=DEMO)
run("v4_demo_collision", dict(trigger_distance="20"), base=DEMO)
run("v4_demo_safe", dict(trigger_distance="40"), base=DEMO)
run("v4_demo_oracle", dict(trigger_distance="20", oracle_mode="visible",
                           oracle_brake="1.0", oracle_delay="0.0"), base=DEMO)

print("done: wrote routes to", ROUTES)
