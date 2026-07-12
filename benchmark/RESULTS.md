# PR #21 performance log

One row per measured change. All runs: `benchmark/full_stack_replay.sh` on the
fixed input bag `~/olive_ws/bags/maze_1loop` (202.7 s single maze loop), stack
pinned to P-cores, debug topics off.

**Machine:** Intel i9-14900HX (hybrid: P-cores 0-15, E-cores 16-31), 32 threads,
governor `powersave` (unchanged), Ubuntu 24.04, ROS 2 Jazzy, gcc 13.3.

**Metrics:** CPU-seconds per process over the replay (`/proc/<pid>/stat`,
hybrid-safe); peak RSS; ATE RMSE + local-stream max step from
`analyze_bag.py` (guardrails — must not regress).

## Runs

| id | commit | change | fusion CPU-s | vo CPU-s | whycon CPU-s | total | ATE RMSE | max step | notes |
|----|--------|--------|-------------:|---------:|-------------:|------:|---------:|---------:|-------|
| B0 | 99dd54c | baseline, `-O2` no `-march` (pre-existing build) | 23.0 | 19.3 | 3.7 | 46.0 | 0.080 | 0.043 | wall 205.8 s; RSS 90/91/100 M; VO ratio 0.96; anchor at 45.6 s. Reference guardrails for the whole series. |

## Micro-benchmarks

(recorded per Phase-4 change; ns/iter original vs optimized, bit-identical assert)

## Environment changes

Installs performed on this machine for the benchmark/verification work, so
there is a permanent record. (Pre-existing: google-benchmark 1.8.3, sysstat,
linux-tools/perf.)

| date | package | version | command | why |
|------|---------|---------|---------|-----|
| — | (none yet) | | | |
