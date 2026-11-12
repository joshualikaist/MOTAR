"""S4: generate the navrl_quad_s4 Gazebo model and PX4 airframe from navrl_quad (plan stage S4).

navrl_quad_s4 is navrl_quad (airframe 10090, its SDF byte for byte) plus three rendering-only sensors on base_link,
matching MOTAR's sensor contract where Gazebo allows:
  - motar_lidar: gpu_ray, 72 x 4 samples, azimuth -175..180 deg, elevation -10..+20 deg, range 0.2-12 m, 20 Hz
    (navrl_lidar_config; MOTAR's LiDAR is yaw-level, this one is body-fixed like a real one);
  - motar_rgb / motar_depth: 160 x 90, horizontal FOV 87 deg, mount (0.10, 0, 0.03) m, 20 Hz, far 20 m
    (navrl_task_config vision; Gazebo pixels are square, so the vertical FOV is 56.2 deg, not MOTAR's 58).
Sensors add no mass or collision. Every <visual> of the vehicle is removed, so its own sensors cannot see its body
or propellers; physics is unchanged. The airframe 10092 is 10090's file with only its @name line changed.

  tools/sitl_s4_model.py --write    # writes into the PX4 tree (outside every repository); prints SHA-256s
"""
import argparse
import hashlib
import math
import pathlib
import re
import sys

PX4 = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot")
MODELS = PX4 / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
AIRFRAMES = PX4 / "ROMFS/px4fmu_common/init.d-posix/airframes"
SRC_MODEL, MODEL = "navrl_quad", "navrl_quad_s4"
SRC_AIRFRAME, AIRFRAME = "10090_gazebo-classic_navrl_quad", "10092_gazebo-classic_navrl_quad_s4"
HFOV = math.radians(87.0)
SENSORS = """      <sensor name='motar_lidar' type='gpu_ray'>
        <pose>0 0 0 0 0 0</pose>
        <update_rate>20</update_rate>
        <always_on>1</always_on>
        <visualize>0</visualize>
        <ray>
          <scan>
            <horizontal><samples>72</samples><resolution>1</resolution><min_angle>%.8f</min_angle><max_angle>%.8f</max_angle></horizontal>
            <vertical><samples>4</samples><resolution>1</resolution><min_angle>%.8f</min_angle><max_angle>%.8f</max_angle></vertical>
          </scan>
          <range><min>0.2</min><max>12.0</max><resolution>0.001</resolution></range>
        </ray>
      </sensor>
      <sensor name='motar_rgb' type='camera'>
        <pose>0.10 0 0.03 0 0 0</pose>
        <update_rate>20</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>%.8f</horizontal_fov>
          <image><width>160</width><height>90</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>60</far></clip>
        </camera>
      </sensor>
      <sensor name='motar_depth' type='depth'>
        <pose>0.10 0 0.03 0 0 0</pose>
        <update_rate>20</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>%.8f</horizontal_fov>
          <image><width>160</width><height>90</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>20</far></clip>
        </camera>
      </sensor>
""" % (math.radians(-175.0), math.radians(180.0), math.radians(-10.0), math.radians(20.0), HFOV, HFOV)


VISUAL = re.compile(r"<visual\b.*?</visual>", re.S)


def model_sdf(src_text, *, sensor_mode='rgbd_lidar'):
    """navrl_quad's SDF with the three sensors on base_link and every <visual> removed: Gazebo's cameras and gpu_ray
    render visuals, so the body mesh and propeller discs would otherwise be seen by the vehicle's own sensors
    (S3c/S4 review r1 BLOCKER). Collisions, inertias, joints and plugins are unchanged."""
    if sensor_mode not in ('rgbd', 'rgbd_lidar'):
        raise ValueError('sensor_mode must be rgbd or rgbd_lidar')
    model = MODEL if sensor_mode == 'rgbd_lidar' else 'navrl_quad_rgbd'
    sensors = SENSORS
    if sensor_mode == 'rgbd':
        sensors = re.sub(r"\s*<sensor name='motar_lidar'.*?</sensor>", '', sensors, count=1, flags=re.S)
    if src_text.count("<model name='%s'>" % SRC_MODEL) != 1:
        raise ValueError("source SDF: expected one <model name='%s'>" % SRC_MODEL)
    n_vis = len(VISUAL.findall(src_text))
    if n_vis == 0:
        raise ValueError("source SDF: no <visual> found; the stripping would be vacuous")
    src_text = VISUAL.sub("", src_text)
    if "<visual" in src_text:
        raise ValueError("a <visual> survived the stripping")
    start = src_text.find("<link name='base_link'>")
    if start < 0 or src_text.count("<link name='base_link'>") != 1:
        raise ValueError("source SDF: expected one base_link")
    end = src_text.find("</link>", start)
    out = src_text[:end] + sensors + "    " + src_text[end:]
    return out.replace("<model name='%s'>" % SRC_MODEL, "<model name='%s'>" % model, 1)


def airframe(src_text, *, sensor_mode='rgbd_lidar'):
    if sensor_mode not in ('rgbd', 'rgbd_lidar'):
        raise ValueError('sensor_mode must be rgbd or rgbd_lidar')
    lines = src_text.splitlines(keepends=True)
    out = []
    for ln in lines:
        if ln.startswith("# @name "):
            ln = ("# @name navrl_quad with MOTAR S4 Gazebo sensors (same airframe as 10090)\n"
                  if sensor_mode == 'rgbd_lidar' else
                  "# @name navrl_quad with MOTAR RGB-D only sensors (same airframe as 10090)\n")
        out.append(ln)
    return "".join(out)


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def write(*, sensor_mode='rgbd_lidar'):
    if sensor_mode not in ('rgbd', 'rgbd_lidar'):
        raise ValueError('sensor_mode must be rgbd or rgbd_lidar')
    model = MODEL if sensor_mode == 'rgbd_lidar' else 'navrl_quad_rgbd'
    af = AIRFRAME if sensor_mode == 'rgbd_lidar' else '10093_gazebo-classic_navrl_quad_rgbd'
    d = MODELS / model
    d.mkdir(exist_ok=True)
    (d / (model + ".sdf")).write_text(model_sdf((MODELS / SRC_MODEL / (SRC_MODEL + ".sdf")).read_text(),
                                              sensor_mode=sensor_mode))
    cfg = (MODELS / SRC_MODEL / "model.config").read_text().replace(SRC_MODEL + ".sdf", model + ".sdf")
    cfg = cfg.replace("<name>%s</name>" % SRC_MODEL, "<name>%s</name>" % model)
    (d / "model.config").write_text(cfg)
    (AIRFRAMES / af).write_text(airframe((AIRFRAMES / SRC_AIRFRAME).read_text(), sensor_mode=sensor_mode))
    cm = AIRFRAMES / "CMakeLists.txt"
    t = cm.read_text()
    if "\t%s\n" % af not in t:
        t = t.replace("\t%s\n" % SRC_AIRFRAME, "\t%s\n\t%s\n" % (SRC_AIRFRAME, af), 1)
        cm.write_text(t)
    tg = PX4 / "src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
    t = tg.read_text()
    if "\t\t%s\n" % model not in t:
        tg.write_text(t.replace("\t\t%s\n" % SRC_MODEL, "\t\t%s\n\t\t%s\n" % (SRC_MODEL, model), 1))
    for p in (d / (model + ".sdf"), d / "model.config", AIRFRAMES / af):
        print(sha(p), p)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', required=True, action='store_true')
    parser.add_argument('--sensor-mode', choices=('rgbd_lidar', 'rgbd'), default='rgbd_lidar')
    args = parser.parse_args()
    write(sensor_mode=args.sensor_mode)
