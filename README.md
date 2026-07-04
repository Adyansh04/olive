# OLIVE

**O**ptimization of **L**idar, **I**nertial, **V**ision & **E**ncoders — graph-based multi-modal sensor fusion for a planar ground robot. ROS 2 **Jazzy**, C++17, [GTSAM](https://github.com/borglab/gtsam) factor-graph backend.

## Architecture

One incremental (iSAM2) keyframe pose graph fuses every modality; each source is a runtime toggle in `config/fusion.yaml`:

| Modality | Role | Enters the graph as |
|----------|------|---------------------|
| LiDAR + IMU (`lio`) | backbone odometry: curvature features → scan-to-map Gauss-Newton, gyro-seeded | between factor per keyframe |
| Wheel odometry (`wheel`) | metric-scale anchor; owns `odom→base` (REP-105) | between factor (tight x/y, loose yaw) |
| WhyCode fiducials (`markers`) | global anchoring / drift reset — **position-only** (x, y, z; orientation is never used) | robust unary anchor factor |
| Monocular camera (`vo`) | auxiliary wheel-scaled KLT visual odometry (default off) | robust between factor |

A soft planar prior pins z / roll / pitch (ground motion). The fused estimate is published on `/olive/odometry` in the `map` frame; the node broadcasts only the `map→odom` correction, so global updates never teleport the robot in the odom frame. Known markers act like local GPS fixes: the `X(0)` prior is deliberately loose, and the first accepted marker sighting bends the whole trajectory into the surveyed world frame.

The LiDAR-inertial core follows the architecture of [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), used as a design reference; the implementation here is written for this project (organized 16-ring clouds, eigen-based plane fits with a collinearity gate, planar-motion handling, coarse-azimuth feature windows) and heavily modified around the marker-anchoring backend. Marker detection uses [whycode](https://github.com/Adyansh04/whycode).

## Nodes

- **`fusion_node`** — the core (`rclcpp_lifecycle`, self-managed bring-up): LiDAR pipeline, factor graph, wheel/marker/VO fusion, TF.
- **`vo_node`** — monocular VO front-end: KLT tracking + essential matrix, yaw from in-place rotation, translation scaled by wheel motion (monocular scale is unobservable on a planar robot). Publishes `/olive/visual_odom`.
- **`whycon`** (from `whycode_vision`) — marker detector, launched when `markers` is enabled; sim configs live in `config/whycode_detector_sim.yaml`.

## Build & run

```zsh
source /opt/ros/jazzy/setup.zsh
cd ~/olive_ws
colcon build --symlink-install --packages-select olive
source install/setup.zsh

# simulation (separate terminal): ros2 launch olive_sim simulation.launch.py world_name:=maze headless:=true
ros2 launch olive sensor_fusion.launch.py
```

The launch reads `config/fusion.yaml`, starts only the enabled modalities, and passes each node its parameter section. `whycode_vision` needs the [Simd](https://github.com/ermig1979/Simd) library (build to a prefix and pass `-DSIMD_INCLUDE_DIR`/`-DSIMD_LIBRARY` if it is not in `/usr/local`).

## Configuration

Everything lives in [config/fusion.yaml](config/fusion.yaml): modality toggles, topics, extrinsics (base←lidar, base←camera), feature/matcher/keyframe tuning, factor noise sigmas (ROS axis order — permuted to GTSAM tangent order internally), the known-marker map, and gating (range window, tracking persistence). Notes that came out of integration testing are recorded as comments next to the parameters they affect, including:

- the detector reports marker positions in the **camera_link body convention** (x forward), not the optical frame;
- decoded sim marker IDs are model index + 1;
- distant markers can mis-decode with `id_valid=true` — the range gate rejects them;
- detected range runs ~5 % long in sim (tune `outer_diameter` in the detector config).

## Real-robot bring-up

`config/fusion_real.yaml` and `config/whycode_detector_real.yaml` are
hardware templates with TODO-marked calibration values. The order that works:

1. **Camera intrinsics** — calibrate, save next to the detector config and
   point `camera.config_path` at it (distortion is supported).
2. **Extrinsics** — prefer `extrinsics_from_tf: true` with your URDF; note
   `camera_frame` must be the frame the detector reports in (real whycode:
   the optical convention, unlike sim).
3. **IMU units/axes sanity** — launch and read the `IMU init` log lines: they
   check |accel| vs 9.8 (g vs m/s²), bias magnitude (deg/s vs rad/s) and
   gravity tilt (mounting rotation). Keep the robot stationary for the
   `imu_init_duration_s` window after launch.
4. **Time offsets** — read the per-sensor `stamp-to-arrival latency` startup
   log lines; compensate via `*_time_offset_s` (LiDAR is the reference).
5. **Detector range scale** — `ros2 run olive calibrate_marker_range.py`
   against a marker at a surveyed position; set `outer_diameter_multiplier`.
6. Survey marker world positions (z is in the MAP frame — origin at
   base_link start height, not the floor) into `known_marker_positions`.

Real LiDAR notes: unorganized (`height==1`) clouds need a `ring` field;
per-point time fields (`time`/`t`/`timestamp`) enable gyro deskew.
LiDAR dropouts are coasted on wheel odometry with `/diagnostics` reporting;
loop closure corrects drift on revisits when markers are out of view.

## Tests

```zsh
colcon test --packages-select olive   # covariance conventions, scan matcher on
                                      # synthetic geometry, marker anchor factor
                                      # (analytic vs numerical Jacobians, drift recovery)
```

## Results (Gazebo Harmonic, maze world, headless)

- LiDAR-inertial core: 10 Hz, 6–12 ms/scan; 35 s multi-turn run **1.1 cm / 0.13°** endpoint error vs ground truth, ~1 mm stationary jitter.
- With wheel factors + planar prior: **1.1 cm / 0.30°**, full REP-105 TF tree.
- Marker drift reset: an 8.49 m map-frame offset snaps to **0.06 m absolute** on the first marker sighting; full-trajectory ATE after anchoring: **0.061 m RMSE, max 0.113 m** (with loop closure active).
- Loop closure: under artificially induced drift (crippled matcher, no wheel factors), an out-and-back route improves from 4.44 m to **0.39 m** final error (11×).
- LiDAR outage (fault-injected): output and TF continue on wheel coasting with `/diagnostics` reporting, cross-track and yaw re-lock on recovery; 10-minute endurance drive with bounded cloud storage ends 1–2 cm from ground truth.
