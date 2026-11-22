#!/usr/bin/env python3
"""Generate minimal ghost-pedestrian route XMLs for the LEAD evaluator.

Geometry (Town01, straight southbound segment at x=396.3):
  ego start y=300 -> trigger point y=260 -> crossing y=208 (crossing_ahead=52m)
  occluder: right shoulder, 5m before crossing; walker dashes east->west.
Sweep variable: ego-to-crossing trigger distance (continuous grid).
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTES = os.path.join(HERE, "..", "routes")
os.makedirs(ROUTES, exist_ok=True)

WEATHER = '''    <weathers>
      <weather route_percentage="0" cloudiness="5.0" precipitation="0.0"
               precipitation_deposits="0.0" wetness="0.0" wind_intensity="0.0"
               sun_azimuth_angle="180.0" sun_altitude_angle="60.0"
               fog_density="0.0" fog_distance="0.0" />
    </weathers>'''

WAYPOINTS = [300, 270, 240, 210, 180, 150]
WAYPOINTS_XML = "    <waypoints>\n" + "\n".join(
    '      <position x="396.3" y="{}" z="0.0" />'.format(y) for y in WAYPOINTS
) + "\n    </waypoints>"


def scenario_block(name, params):
    attrs = " ".join('{}="{}"'.format(k, v) for k, v in params.items())
    return (
        '      <scenario type="GhostPedestrian" name="{name}">\n'
        '        <trigger_point x="396.3" y="260" z="1.0" yaw="270" />\n'
        '        <ghost_params {attrs} />\n'
        '      </scenario>'
    ).format(name=name, attrs=attrs)


def write_route(fname, scenarios):
    body = "\n".join(scenarios) if scenarios else ""
    xml = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<routes>\n'
        '  <route id="0" town="Town01">\n'
        '{weather}\n'
        '{waypoints}\n'
        '    <scenarios>\n'
        '{scenarios}\n'
        '    </scenarios>\n'
        '  </route>\n'
        '</routes>\n'
    ).format(weather=WEATHER, waypoints=WAYPOINTS_XML, scenarios=body)
    path = os.path.join(ROUTES, fname)
    with open(path, "w") as f:
        f.write(xml)
    print("wrote", path)


BASE = dict(
    walker_speed="1.5", crossing_ahead="52", occluder_lateral="2.8",
    occluder_before="5", walker_lateral="3.2", walker_end_lateral="-2.0",
    end_after="20", seed="0", save_frames="1",
)
OUTROOT = "/root/autodl-tmp/ghost_runs"

# 1) Baseline: no scenario at all
write_route("baseline.xml", [])

# 2) Occluder only: parked car, no pedestrian
p = dict(BASE, walker_enabled="0", output_dir=OUTROOT + "/occluder_only")
write_route("occluder_only.xml", [scenario_block("OccluderOnly", p)])

# 3) Ghost pedestrian: continuous fine scan of trigger distance.
#    Coarse anchors 25/15/9 + 2 m grid between 15 and 25 to localise the
#    safe->collision transition. Add values here to extend the sweep.
DISTANCES = (25, 24.5, 24.25, 24, 23, 22, 20, 18, 16, 15, 9)
for d in DISTANCES:
    # "24.5" -> name "GP_24p5m", file "gp_24p5m.xml", run dir "gp_24p5m"
    tag = str(d).replace(".", "p")
    name = "GP_{}m".format(tag)
    p = dict(BASE, walker_enabled="1", trigger_distance=str(d),
             output_dir=OUTROOT + "/gp_{}m".format(tag))
    write_route("{}.xml".format(name.lower()), [scenario_block(name, p)])

# Repeatability: independent second runs at the two coarse anchors.
for d, suffix in ((24, "24m_r2"), (25, "25m_r2")):
    name = "GP_{}".format(suffix)
    p = dict(BASE, walker_enabled="1", trigger_distance=str(d),
             output_dir=OUTROOT + "/gp_{}".format(suffix))
    write_route("{}.xml".format(name.lower()), [scenario_block(name, p)])

print("done")
