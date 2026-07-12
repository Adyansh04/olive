# olive performance benchmarks

Measurement tooling for the PR #21 performance work. Every optimization commit
is validated here **before** it lands; numbers are logged in [RESULTS.md](RESULTS.md).

## Full-stack benchmark

```zsh
benchmark/full_stack_replay.sh <label> [--perf]
```

Replays the fixed single-loop sensor bag `~/olive_ws/bags/maze_1loop` (202.7 s,
recorded once — never re-recorded) through the live stack (`fusion_node` +
`whycon` + `vo_node`, debug off) with **no simulator running**, and reports:

- **CPU-seconds + peak RSS per process** — read from `/proc/<pid>/stat`
  (utime+stime), which is robust on hybrid P/E-core CPUs where perf-event
  counts split across `cpu_core`/`cpu_atom` and are awkward to compare.
- **ATE RMSE + local-stream continuity** — the stack's live outputs are
  captured to a small temp bag each run and fed to
  `scripts/evaluation/analyze_bag.py`; these are the correctness guardrails
  (an optimization must not move them).

Stability measures: the stack is pinned to P-cores, the replay/record infra to
E-cores; same input bag every run; CPU governor printed in the log. Artifacts
land in `~/olive_ws/perf_data/bench_runs/<label>/`. `--perf` additionally
captures a `perf record` profile of the stack during the replay.

## Micro-benchmarks (`micro/`)

Google-Benchmark harnesses for individual hot functions (scan matcher align,
keyframe-map local-map build). Each is self-contained and built by
`micro/build_and_run.sh` with the package's own include paths and compile
flags (mirrored from `compile_commands.json`), so A/B numbers match the real
build. Used to compare original vs optimized implementations **before** a
replacement is committed; each bench also asserts the optimized output is
bit-identical to the original.
