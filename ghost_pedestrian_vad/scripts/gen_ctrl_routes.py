#!/usr/bin/env python3
"""Extra routes: 4-way factor controls + brake-hold/hysteresis variants."""
import os

OUT = os.path.expanduser("~/Work/XXL/vad_ghost/runs/ghost_vad")
TPL = """<?xml version="1.0" encoding="UTF-8"?>
<routes>
  <route id="0" town="Town05_Opt">
    <weathers>
      <weather route_percentage="0" cloudiness="10.0" precipitation="0.0"
               precipitation_deposits="0.0" wetness="0.0" wind_intensity="0.0"
               sun_azimuth_angle="180.0" sun_altitude_angle="45.0"
               fog_density="0.0" fog_distance="0.0" />
    </weathers>
    <waypoints>
      <position x="150.6" y="-97.8" z="0.0" />
      <position x="151.4" y="-78.2" z="0.0" />
      <position x="151.5" y="-58.2" z="0.0" />
      <position x="151.5" y="-38.2" z="0.0" />
      <position x="151.5" y="-14.2" z="0.0" />
    </waypoints>
    <scenarios>
      <scenario type="GhostPedestrianV4" name="GP4_vad_{tag}">
        <trigger_point x="150.6" y="-97.8" z="0.0" yaw="80" />
        <ghost_params trigger_distance="20" walker_speed="1.5" crossing_ahead="55"
                      occluder_nose="1.2" walker_start_off="4.0" walker_end_off="-5.0"
                      end_after="18" save_frames="0" seed="0"
                      walker_enabled="{walker}" occluder_enabled="{occ}"
                      output_dir="{out}" />
      </scenario>
    </scenarios>
  </route>
</routes>
"""


def gen(tag, walker=1, occ=1):
    xml = TPL.format(tag=tag, walker=walker, occ=occ,
                     out=os.path.join(OUT, "run_" + tag))
    p = os.path.join(OUT, "vad_%s.xml" % tag)
    open(p, "w").write(xml)
    print("wrote", p)


for r in (1, 2, 3):
    gen("empty_d20m_r%d" % r, walker=0, occ=0)      # 空道路
for r in (2, 3):
    gen("occonly_d20m_r%d" % r, walker=0, occ=1)     # 仅遮挡车（r1 已有）
for r in (1, 2, 3):
    gen("hold_d20m_r%d" % r, walker=1, occ=1)        # 遮挡+行人，制动保持
for r in (1, 2, 3):
    gen("hyst_d20m_r%d" % r, walker=1, occ=1)        # 遮挡+行人，滞回
