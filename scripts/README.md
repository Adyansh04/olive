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
| `analyze_bag.py <bag>` | Offline counterpart to `ate_eval.py`: reads a recorded bag (mcap/sqlite3, zstd file-compression handled automatically), prints each odometry stream's frame_id + first position, and the fused-vs-ground-truth error split at the marker anchor with an early-vs-late drift check. |
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

Reference numbers (maze world, headless, after calibration): drive test
~1 cm / 0.3°; ATE post-anchor absolute mean/RMSE 0.07 m, yaw ~0.2°;
anchor snap 8.49 m → 0.06 m.
