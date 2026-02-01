rm_bringup
=======================

Centralized bringup package for rm_localization. All launch files and common parameter YAMLs live here so other backend packages (fast_lio, faster_lio_ros2, point_lio, fast_lio_localization_ros2) do not need git changes.

slam_and_localize.launch.py
-----------------

- Fast-LIO mapping (default):
  `ros2 launch rm_bringup localization_bringup.launch.py backend:=fast_lio`

- FASTER-LIO mapping:
  `ros2 launch rm_bringup localization_bringup.launch.py backend:=faster_lio`

- Point-LIO mapping:
  `ros2 launch rm_bringup localization_bringup.launch.py backend:=point_lio`

- Override params file, enable global localization helpers and RViz:
  `ros2 launch rm_bringup localization_bringup.launch.py backend:=point_lio point_lio_params:=/path/to.yaml run_global_localization:=true rviz:=true`

Parameter YAMLs are under `config/`. Edit them here to keep all robot-specific settings in one place.

Sentry Bringup (All-in-One)
---------------------------

The master launch file to start Driver, Localization, Nav2, Decision, and Communication.

**Run Navigation Mode (Match + Nav):**
```bash
ros2 launch rm_bringup sentry_bringup.launch.py mode:=nav map:=/path/to/my_map.yaml
```

**Run Mapping Mode (Build Map):**
```bash
ros2 launch rm_bringup sentry_bringup.launch.py mode:=mapping backend:=fast_lio
```

**Options:**
- `driver`: `true` (default)
- `comm`: `true` (default)
- `decision`: `true` (default)
- `backend`: `fast_lio` | `faster_lio` | `point_lio` (default: `fast_lio`)

SLAM Mapping Only
-----------------

Launch mapping without localization extras using `slam_mapping_only.launch.py`.

- Live sensors + RViz (Point-LIO by default):
  `ros2 launch rm_bringup slam_mapping_only.launch.py rviz:=true`

- Choose backend explicitly:
  `ros2 launch rm_bringup slam_mapping_only.launch.py backend:=fast_lio`

- Play a rosbag (starts paused, uses simulated clock):
  1) Launch mapping: `ros2 launch rm_bringup slam_mapping_only.launch.py rviz:=true`
  2) In another terminal, manually play: `ros2 bag play rosbags/my_bag --clock --start-paused`

- Record LiDAR and IMU while mapping:
  `ros2 launch rm_bringup slam_mapping_only.launch.py record_rosbag:=true record_output:=rosbags/mapping_record`

- Override backend parameter files:
  - Fast-LIO: `fast_lio_params:=/path/to.yaml`
  - FASTER-LIO: `faster_lio_params:=/path/to.yaml`
  - Point-LIO: `point_lio_ros2_params:=/path/to.yaml`

Arguments (key ones)
- `backend`: `fast_lio` | `faster_lio` | `point_lio` (default: `point_lio`)
- `rviz`: `true` | `false` (default: `true`)
- `use_sim_time`: `true` | `false` (default: `true`)
- Rosbag playback is manual; this launch file does not run `ros2 bag play`.
- `record_rosbag`: record `/livox/lidar` and `/livox/imu` (default: `false`)
- `record_output`: output prefix/path for recording (default: `rosbags/mapping_record`)
- `pcd_save_en`: enable map PCD saving override (default: `True`)
- `pcd_save_interval`: seconds between periodic saves; `-1` disables (default: `-1`)
- Default params: `config/fast_lio_mid360.yaml`, `config/faster_lio_ros2.yaml`, `config/point_lio_mid360.yaml`

Notes
- Rosbag playback uses `--clock` and `--start-paused` so you can press space to start.
- RViz config is loaded from the Point-LIO package: `point_lio_ros2/rviz_cfg/localize.rviz`.
