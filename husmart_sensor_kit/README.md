# Hüsmart sensor kit v3 (Autoware, AWSIM simulation)

Sensors of the Hüsmart vehicle (v3, 2026-11-11):
- 1 × **Hesai OT128** on the roof (230 m).
- 8 × **Hesai JT128** around the body at bumper height (60 m).
- 2 × **Stereolabs ZED X**, one facing forward and one facing backward.
- 1 × **Xsens MTi-7** (GNSS/INS).

Lidar names, layout, IPs and ports match the lab kit: [HUSMART-AV/Autoware_1.9.0_launch](https://github.com/HUSMART-AV/Autoware_1.9.0_launch). The difference: there, Autoware's `sample_sensor_kit` package is modified; here Autoware is not touched at all.

This folder does **not** modify the Autoware Docker image. It is a package overlay mounted read-only into the container: `husmart_sensor_kit_description` + `husmart_sensor_kit_launch`. The original `awsim_sensor_kit` stays as it is.

## Poses (ESTIMATED — the vehicle has not been measured)

`base_link` is the centre of the rear axle projected onto the ground. Axes: x forward, y left, z up (m). The angle in the direction column is yaw (positive to the left).

| Sensor | Frame / topic name | Position (base_link) | Direction | Real vehicle IP:port |
|---|---|---|---|---|
| Hesai OT128 (roof) | `ot128_top` | (0.90, 0.00, 2.00) | upright, 0° | 192.168.1.201:2368 |
| JT128 front | `jt128_front` | (4.00, 0.00, 0.50) | upright, 0° | 192.168.1.202:2369 |
| JT128 front left / front right | `jt128_front_left/right` | (3.50, ±1.00, 0.50) | upright, ±45° | .203:2370 / .204:2371 |
| JT128 left / right | `jt128_left/right` | (1.00, ±1.05, 0.50) | upright, ±90° | .205:2372 / .206:2373 |
| JT128 rear left / rear right | `jt128_rear_left/right` | (−0.80, ±1.00, 0.50) | upright, ±135° | .207:2374 / .208:2375 |
| JT128 rear | `jt128_rear` | (−1.10, 0.00, 0.50) | upright, 180° | .209:2376 |
| ZED X front (between the two cameras) | `zedx_front_camera_center` | (1.65, 0.00, 1.80) | 5° down | |
| ZED X rear | `zedx_rear_camera_center` | (−0.15, 0.00, 1.76) | backward, 5° down | |
| ZED X eyes | `zedx_{front,rear}_{left,right}_camera_optical_frame` | ±0.06 from centre (baseline 0.12 m) | optical (z forward) | |
| Xsens MTi-7 | `mti7_imu_link` | (1.20, 0.00, 0.45) | aligned with the body | |
| GNSS antenna | `gnss_link` | (0.40, 0.00, 1.82) | — | |

Known limitation: the JT128s are mounted upright at 0.5 m, so they only see above −3.5°; the roof OT128 reaches the ground only beyond ~4 m. As a result, low objects (below ≈0.4 m) within ~2 m of the body are not seen; person- and vehicle-height objects are (measured 2026-11-11).

In simulation the OT128 is produced by AWSIM's `HesaiPandar128E4X` model (the OT128 equivalent): 0.2° horizontal resolution, 230 400 rays per scan.

JT128 horizontal resolution in simulation is 0.8°. The real sensor gives 0.4°; it was reduced because of the simulation load.

To change the poses, edit **both places together**; the values must match:
1. **Autoware:** `share/husmart_sensor_kit_description/config/sensors_calibration.yaml` and `urdf/sensors.xacro`.
2. **AWSIM:** the `"SensorRig"` section of `awsim_ayar/beytepe-config-husmart.json` (repo root). AWSIM does not need to be rebuilt.

## Processing pipeline

- **Lidar preprocessing:** Autoware's common pipeline runs for each lidar: self-body crop (`config/self_body_crop.param.yaml`: the vehicle body and bodywork seen by the side lidars), mirror crop, motion distortion correction, ring outlier filter. The driver is off; AWSIM publishes the points.
- **Concatenation:** the 9 clouds are merged into `/sensing/lidar/concatenated/pointcloud` (used by perception and obstacle detection).
- **Ring outlier filter:** `config/ring_outlier_filter_node.param.yaml`, `max_rings_num: 129`. AWSIM numbers the OT128 rings 1..128; with the default of 128 the filter crashes. The fix comes from the lab kit.
- **Enabling / disabling lidars:** the `enable_ot128_top`, `enable_jt128_front` ... arguments (all on by default). A disabled lidar is missing from the concatenation; the concatenation waits 0.2 s and publishes with the rest.
- **Localization (NDT):** uses only the roof OT128 (`/sensing/lidar/ot128_top/pointcloud_before_sync`), with no concatenation delay. Why: the roof lidar has an unobstructed 360° long-range view. The bumper lidars mostly see the nearby ground and vehicles; with the concatenated cloud, NDT's 1500-point downsample fills up with those near points. This is set by `override/tier4_localization_component.launch.xml`; if that file is not mounted into the Autoware container, NDT uses the concatenated cloud.
- **Driving IMU:** MTi-7 (`imu.launch.xml`, input `/sensing/imu/mti7/imu_raw`).

## Topics (published by AWSIM)

| Topic | Type | Rate |
|---|---|---|
| `/sensing/lidar/{ot128_top,jt128_front,...}/pointcloud_raw_ex` | PointCloud2 (XYZIRCAEDT; OT128 rings 1..128, JT128 0..127) | 10 Hz |
| `/sensing/camera/zedx_{front,rear}/{left,right}/image_rect_color` + `camera_info` | Image bgr8 1920x1200 | 15 Hz |
| `/sensing/imu/mti7/imu_raw` | Imu (orientation + angular velocity + acceleration) | 100 Hz |
| `/sensing/gnss/mti7/nav_sat_fix` | NavSatFix (real latitude/longitude) | 4 Hz |
| `/sensing/ins/mti7/pose_with_covariance` | INS pose (map frame) | 50 Hz |
| `/sensing/gnss/pose_with_covariance` | antenna pose (Autoware initial pose) | 1 Hz |

The right camera's `camera_info` P[3] is −fx·B, so stereo_image_proc can use it directly.

## Running

Use `run_awsim.sh` and `run_autoware_beytepe.sh` from the repo root (see the main README). What `run_autoware_beytepe.sh` does for this kit:
- Mounts the kit: `-v husmart_sensor_kit:/opt/husmart:ro`.
- Switches localization to the roof OT128:
  `-v husmart_sensor_kit/override/tier4_localization_component.launch.xml:/opt/autoware/share/autoware_launch/launch/components/tier4_localization_component.launch.xml:ro`
- Runs `export AMENT_PREFIX_PATH=/opt/husmart:$AMENT_PREFIX_PATH` before launching.
- Passes `sensor_model:=husmart_sensor_kit` as a launch argument.

To go back to the original Lexus sensors, start AWSIM with `awsim_ayar/beytepe-config.json` and Autoware with `./run_e2e_sim.sh`.

## Moving to the real vehicle

- Write the measured mounting poses into the calibration files, or obtain them by calibration.
- `lidar.launch.xml`: IPs and ports are filled in from the lab table, but the driver is off (`launch_driver` false).
  - OT128: works with the `Pandar128E4X` model of Nebula in the Autoware image.
  - JT128: Nebula has **no** JT128 support (the lab used `PandarQT128` as a stand-in, which does not decode JT128 packets correctly). Hesai's own driver, HesaiLidar_ROS_2.0, is needed.
  - The output must be `/sensing/lidar/NAME/pointcloud_raw_ex`. The ring outlier filter uses the `channel` and `azimuth` fields.
- `sensing.launch.xml`: add zed_wrapper and remap the images to `/sensing/camera/zedx_*/...`. Add xsens_mti_ros2_driver and remap `/imu/data` to `/sensing/imu/mti7/imu_raw`.
- GNSS: the map's `map_projector_info.yaml` is `Local`. Converting NavSatFix into the map frame (gnss_poser) needs a `TransverseMercator` projector (origin 39.87116440956932 N, 32.7356890084472 E). This changes how the map is loaded and must be tested separately.
