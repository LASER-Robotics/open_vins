# OpenVINS for LASER

OpenVINS estimates motion from camera images and IMU measurements using a filter-based visual-inertial estimator. This LASER fork supplies the OpenVINS implementation used by the LUS VIO integration.

Its MSCKF sliding window uses visual feature tracks to constrain the inertial state. The code also supports persistent SLAM features, camera/IMU calibration, static and dynamic initialization, and zero-velocity updates. Which features run depends on the selected profile.

## Packages

| Package | Role |
| --- | --- |
| `ov_core` | Feature tracking, geometry and shared estimation utilities. |
| `ov_init` | Visual-inertial initialization. |
| `ov_msckf` | State estimator, ROS interfaces and simulator. |
| `ov_eval` | Trajectory evaluation tools. |
| `ov_data` | Dataset-related resources. |

The LUS launch and calibration profiles are in the separate `laser_open_vins_core` package. `laser_vins_republisher` converts the estimator output for the estimation manager.

## Build and run

From a ROS 2 workspace with the OpenVINS dependencies installed:

```bash
colcon build --packages-up-to ov_msckf
source install/setup.bash
```

For LUS, start the estimator through its integration launch:

```bash
export UAV_NAME=uav1
ros2 launch laser_open_vins_core open_vins.launch.py namespace:=uav1 open_vins_params_folder:=rs_244622073910
```

Pass the profile for the actual camera and IMU. Camera drivers, IMU filtering and odometry republishing run separately. `REAL_UAV=false` selects simulation time in the LASER launch.

## Interfaces

The LUS launch starts `ov_msckf/run_subscribe_msckf` with node name `ov_msckf`. For namespace `uav1`:

| Direction | Topic or parameter | Type | Purpose |
| --- | --- | --- | --- |
| Input | Topic named by `topic_imu` | `sensor_msgs/msg/Imu` | Inertial measurements. |
| Input | Topics named by `topic_camera0`, `topic_camera1` | `sensor_msgs/msg/Image` | Camera images, according to the configured camera count. |
| Output | `/uav1/ov_msckf/odomimu` | `nav_msgs/msg/Odometry` | Estimated IMU state. |
| Output | `/uav1/ov_msckf/poseimu` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Pose and covariance. |
| Output | `/uav1/ov_msckf/pathimu` | `nav_msgs/msg/Path` | Estimated path. |
| Output | `/uav1/ov_msckf/points_msckf`, `points_slam`, `points_aruco` | `sensor_msgs/msg/PointCloud2` | Feature clouds under the same node prefix. |
| Output | `/uav1/ov_msckf/num_points_slam`, `num_points_msckf` | `std_msgs/msg/Float64` | Feature counts under the same node prefix. |
| Output | `/uav1/ov_msckf/status_num_points_slam` | `std_msgs/msg/String` | SLAM feature status. |

The visualizer also provides ground-truth and loop-closure-related outputs when those paths are used. In LUS, the republisher subscribes to `odomimu` and publishes `/<UAV_NAME>/vins_republisher/odometry`.

## Configuration

The ROS parameter `config_path` selects the estimator's OpenCV YAML file. The LASER wrapper builds this path from `open_vins_params_folder`, then passes `topic_imu`, `topic_camera0`, `topic_camera1`, `verbosity`, `namespace` and `use_sim_time` as ROS parameters.

| Profile setting | Meaning |
| --- | --- |
| `use_stereo`, `max_cameras` | Camera count and stereo constraints. |
| `use_fej`, `integration` | First-estimate Jacobians and integration method. |
| `max_clones` | Sliding-window state count. |
| `max_slam`, `max_slam_in_update`, `max_msckf_in_update` | Limits for persistent features and measurement updates. |
| `feat_rep_*` | Feature parameterizations used by the different update paths. |
| `calib_cam_*`, `calib_imu_*` | Online intrinsic, extrinsic and time-offset calibration options. |
| `init_*`, `zupt_*`, `try_zupt` | Initialization and zero-velocity updates. |
| `use_klt`, `num_pts`, `fast_threshold`, `grid_*`, `min_px_dist` | Visual tracking method, feature count and spatial distribution. |
| `track_frequency`, `downsample_cameras`, `histogram_method`, `num_opencv_threads` | Tracking rate, image processing and CPU settings. |
| `up_*`, `fi_*` | Measurement-update noise/thresholds and feature-initialization checks. |
| `gravity_mag` | Gravity magnitude in m/s². |
| `relative_config_imu`, `relative_config_imucam` | IMU and camera/IMU calibration files relative to the profile. |
| `record_timing_*`, `save_total_state`, `filepath_*` | Timing logs and saved state outputs. |

Use camera intrinsics, distortion, extrinsics, timing and IMU noise calibrated for the sensor setup. See the `laser_open_vins_core` README for the supplied LUS profiles and launch arguments.

## Evaluation and upstream documentation

OpenVINS includes trajectory metrics such as ATE, RPE, NEES and RMSE, along with a visual-inertial simulator and dataset tools. The [documentation](https://docs.openvins.com/) and [getting-started guide](https://docs.openvins.com/getting-started.html) cover the wider project. Historical releases and related extensions are available in the [upstream repository](https://github.com/rpng/open_vins).

## Reference and license

OpenVINS was written by the Robot Perception and Navigation Group (RPNG) at the University of Delaware. Cite the following publication when using it in research:

```txt
@Conference{Geneva2020ICRA,
  Title      = {{OpenVINS}: A Research Platform for Visual-Inertial Estimation},
  Author     = {Patrick Geneva and Kevin Eckenhoff and Woosik Lee and Yulin Yang and Guoquan Huang},
  Booktitle  = {Proc. of the IEEE International Conference on Robotics and Automation},
  Year       = {2020},
  Address    = {Paris, France},
  Url        = {\url{https://github.com/rpng/open_vins}}
}
```

The code and documentation retain the [GNU General Public License v3](LICENSE). Preserve copyright and license notices when redistributing modified versions.
