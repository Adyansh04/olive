# OLIVE

**O**ptimization of **L**idar, **I**nertial, **V**ision & **E**ncoders — graph-based multi-modal sensor fusion for a planar ground robot. ROS 2 **Jazzy**, C++17, [GTSAM](https://github.com/borglab/gtsam) factor-graph backend.

One iSAM2 keyframe graph fuses LiDAR, IMU (tightly coupled), wheel encoders and WhyCode fiducials, with ICP loop closure. It produces both a globally-accurate map-frame pose and a smooth, jump-free odom-frame stream — a drop-in localization backend for Nav2.

<p align="center">
<img src="media/trajectory.png" width="49%" alt="Fused trajectory vs ground truth over 3 maze loops (3.6 cm RMSE)">
<img src="media/local_vs_wheel.png" width="49%" alt="Smooth local odometry stays tight while raw wheel odometry drifts">
</p>

*Left: the fused estimate tracks ground truth to a few cm over three 56 m loops. Right: in the continuous odom frame, OLIVE's fused local odometry (green) stays a tight, repeatable square while raw wheel dead-reckoning (red) drifts 17 m — the "smoother, more accurate odometry" a controller consumes.*

## Architecture

One incremental (iSAM2) keyframe pose graph fuses every modality; each source is a runtime toggle in `config/fusion.yaml`:

| Modality | Role | Enters the graph as |
|----------|------|---------------------|
| LiDAR + IMU (`lio`) | backbone odometry: curvature features → scan-to-map Gauss-Newton, gyro-seeded | between factor per keyframe |
| IMU (tight coupling) | preintegrated rotation constraint + **online gyro/accel bias estimation** (`imu_preintegration`) | `CombinedImuFactor` chain over `V(i)`/`B(i)` states |
| Wheel odometry (`wheel`) | metric-scale anchor | between factor (tight x/y, loose yaw) |
| WhyCode fiducials (`markers`) | **landmark variables `L(id)`** (TagSLAM-style): surveyed ids anchor the world frame, any other repeatedly-sighted marker acts as an odometry constraint — position-only, orientation never used | robust binary observation factor (+ position prior for surveyed ids) |
| Monocular camera (`vo`) | auxiliary wheel-scaled KLT visual odometry (default off) | robust between factor |

A soft planar prior pins z / roll / pitch (ground motion). Two odometry outputs implement the REP-105 split end-to-end:

- **`/olive/odometry`** (map frame) — the globally-accurate graph estimate; marker anchors and loop closures jump HERE via the `map→odom` correction.
- **`/olive/odometry_local`** (odom frame, ~50 Hz) — a continuous, jump-free stream built from scan-match increments + wheel + gyro, with the matching `odom→base_footprint` TF (`publish_odom_tf`). This is what a local controller (Nav2) should consume: `map→odom` absorbs every global correction, so the local stream never teleports.

Known markers act like local GPS fixes: the `X(0)` prior is deliberately loose, and the first accepted marker sighting bends the whole trajectory into the surveyed world frame.

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

7. **odom→base TF ownership** — `publish_odom_tf: false` keeps your base
   driver's TF (this node only broadcasts `map→odom`). To let Nav2 consume
   the fused local odometry, disable the driver's TF and flip the flag:
   exactly ONE node may publish `odom→base`.
8. **IMU tight coupling** — after the basics work, calibrate the IMU noise
   sigmas (datasheet or Allan variance) and set `imu_preintegration: true`;
   verify by plotting `/olive/debug/bias` (the online estimate should sit
   near the stationary-init value and track slow drift).

Real LiDAR notes: unorganized (`height==1`) clouds need a `ring` field;
per-point time fields (`time`/`t`/`timestamp`) enable gyro deskew.
LiDAR dropouts are coasted on wheel odometry with `/diagnostics` reporting;
loop closure corrects drift on revisits when markers are out of view.

### Nav2 wiring (drop-in AMCL replacement)

This node fills the AMCL slot: it owns `map→odom` (and, with
`publish_odom_tf`, the smooth `odom→base_footprint`). Point the Nav2
controller's `odom_topic` at **`/olive/odometry_local`** and keep the
global/local costmap frames at `map`/`odom` as usual; drop AMCL from the
bringup. Localize before you navigate: until the first surveyed marker is
sighted the map frame is spawn-relative (the first anchor is a one-time
multi-metre `map→odom` jump).

## Tests

```zsh
colcon test --packages-select olive   # covariance conventions, scan matcher on
                                      # synthetic geometry, marker anchor factor
                                      # (analytic vs numerical Jacobians, drift recovery)
```

## Results

Gazebo Harmonic, maze world (16×16 m), 3 loops of a 56 m square. Figures are
generated from a recorded bag by [`scripts/plot_results.py`](scripts/plot_results.py).

| Metric | Value |
|--------|-------|
| Absolute trajectory error (post-anchor, vs ground truth) | **3.6 cm RMSE**, 9.0 cm max |
| Drive-test relative accuracy (35 s multi-turn) | **1.4 cm / 0.22°** |
| First-anchor drift reset (spawn frame → world) | 8.5 m → few cm, one sighting |
| Local-odometry drift, 3 loops (168 m) | **0.9 m** (raw wheel odometry: 17 m) |
| Local-stream max step across the 8.5 m anchor snap | **4.3 cm** (corrections stay in `map→odom`) |
| Unsurveyed-marker landmark convergence | **6–8 cm** from sightings alone |
| LiDAR-core throughput | 10 Hz, 6–12 ms/scan |

### Accuracy over the run — drift-free after the first anchor

![Absolute position error over time](media/error_time.png)

The map frame starts spawn-relative (8.5 m offset); the first fiducial sighting
snaps the whole trajectory into the surveyed world frame, after which error
stays bounded — dipping to millimetres at each corner marker and never
accumulating across the three loops.

### The REP-105 split — smooth local odometry for Nav2

![map→odom absorbs the jumps, the local stream never does](media/continuity.png)

OLIVE publishes two odometry outputs: `/olive/odometry` (map frame,
globally accurate, **allowed** to jump) and `/olive/odometry_local` (odom
frame, ~50 Hz, **continuous**). Every global correction — the 8.5 m anchor
snap, every loop closure — lands in the `map→odom` transform; the local stream
a controller consumes never steps more than a few cm. That is what makes it a
drop-in AMCL replacement for Nav2.

### Tight IMU coupling — online bias & velocity

![Online IMU bias and velocity estimated by the graph](media/imu_state.png)

Velocity and gyro/accel bias are states in the graph, chained by preintegrated
`CombinedImuFactor`s. The estimated velocity tracks the square drive; the gyro
bias stays bounded near zero (the sim IMU has negligible true bias). In a
fault-injection test, a 0.02 rad/s bias stepped on mid-run — invisible to the
stationary startup estimate — is recovered online to **0.0197 rad/s within
~25 s**, something a loosely-coupled filter cannot do.

### Robustness (fault injection)

- **Marker odometry / LiDAR blackout**: during a 25 s LiDAR outage, marker
  observations pull the coasted trajectory back to **1.1 cm** final error;
  with markers off the same outage drifts 4.9 m and never recovers.
- **Corrupted wheel odometry** (`scripts/wheel_odom_relay.py`, ~13 m injected
  drift): the fused output is unchanged — LiDAR + markers carry the estimate.
- **Loop closure** (crippled matcher, no wheel factors): an out-and-back route
  improves from 4.44 m to **0.39 m** final error (11×).
- **Endurance**: a 10-minute continuous drive with bounded cloud storage ends
  1–2 cm from ground truth, with per-sensor `/diagnostics` health throughout.

Reproduce: record with [`scripts/square_drive.py`](scripts/square_drive.py) +
`ros2 bag record`, analyse offline with
[`scripts/analyze_bag.py`](scripts/analyze_bag.py), and regenerate these
figures with `scripts/plot_results.py`. A full replay walkthrough is in
`demo/REPLAY_GUIDE.md`.
