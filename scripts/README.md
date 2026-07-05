# olive test & calibration scripts

Integration-level tools for the fusion stack, developed and used during the
sim verification of every stage. All assume a sourced workspace; the drive
tests additionally assume the simulation and fusion stack are running (use
`bringup_test.sh` to get there from any state).

| Script | Purpose |
|--------|---------|
| `bringup_test.sh` | Kill every leftover sim/fusion process, start one simulation + one fusion stack, verify process counts, optionally run one of the tests below. Duplicate half-dead launches silently corrupt every topic metric — always bring the stack up through this. |
| `drive_test.py [--long]` | Relative accuracy: drives a scripted pattern and compares fused vs ground truth on displacement / heading / path length. Works before marker anchoring (frame offsets cancel). |
| `square_drive.py [--half --loops --speed]` | Closed-loop square driver: chases the four corners of a `±half` square `--loops` times using `/ground_truth` feedback, turning in place at each corner so laps overlap cleanly. Used to record repeatable demo/test bags (paired with `ros2 bag record`). Defaults ride the maze's open outer ring (`--half 7.0`). |
| `ate_eval.py` | Absolute accuracy: long tour including a marker-anchoring event; continuously samples both streams, time-associates them and reports mean/RMSE/max position and yaw error split into pre-anchor (odometry drift) and post-anchor (world-frame) phases. |
| `analyze_bag.py <bag>` | Offline counterpart to `ate_eval.py`: reads a recorded bag (mcap/sqlite3, zstd file-compression handled automatically), prints each odometry stream's frame_id + first position, the fused-vs-ground-truth error split at the marker anchor with an early-vs-late drift check, and the smooth-stream continuity report (`--local-topic`, `--max-step` — the local odom must never jump while map->odom absorbs the corrections). |
| `plot_results.py` | Regenerates the README result figures (`../media/*.png`) from a recorded bag: trajectory-vs-ground-truth, smooth-local-vs-raw-wheel in the odom frame, error-over-time, the REP-105 continuity split, and the online IMU bias/velocity. Point it at a bag with `OLIVE_BAG=<path> ros2 run olive plot_results.py`. |
| `wheel_odom_relay.py` | Wheel-odometry fault injector: republishes `/odom` with distance-scale error, heading drift per metre and per-step noise (`--scale`, `--yaw-drift-deg-per-m`, `--noise-xy`) — point `wheel_odom_topic` at `/odom_noisy` to prove the fusion survives corrupted wheel odom. |
| `imu_bias_relay.py` | Gyro fault injector: republishes `/imu/data` with a constant bias (`--bias-z`), optionally stepped on mid-run (`--bias-after-s`, invisible to the stationary init — exercises the tight coupling's online bias estimation) and/or a stamp offset (`--stamp-offset-s`). |
| `lidar_gate_relay.py` | LiDAR outage injector: relays the cloud topic with a mute window (`--mute-after`, `--mute-duration`) to exercise dropout coasting and marker-held recovery. |
| `marker_demo.py` | The drift-reset demo: reports the absolute error at checkpoints before and after the first marker sighting — the snap from the spawn-frame offset to sub-decimeter world-frame agreement. |
| `calibrate_marker_range.py` | Points the robot at a marker with a known world position, averages the detected range against the true camera-to-marker distance and prints the corrected `outer_diameter_multiplier` for the detector config. Run after changing marker size, camera, or detector settings. |

Typical sessions:

```zsh
# full bring-up + relative drive test in the maze
ros2 run olive bringup_test.sh maze drive

# bring-up with RViz debug visualization, then the drift-reset demo
ros2 run olive bringup_test.sh maze none rviz
ros2 run olive marker_demo.py

# absolute accuracy evaluation
ros2 run olive ate_eval.py

# recalibrate the detector range scale
ros2 run olive calibrate_marker_range.py --current-multiplier 1.096
```

Reference numbers (maze world, headless, full stack: landmarks + tight IMU
coupling + smooth odom): drive test ~1-2 cm / 0.2°; ATE post-anchor RMSE
0.056 m, yaw ~0.2°; anchor snap 8.49 m → 0.06 m absorbed entirely by
map→odom (local stream max step 3-4 cm); free-landmark convergence 6-8 cm;
online gyro-bias step recovery within ~25 s.
