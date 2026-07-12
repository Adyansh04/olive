# PR #21 performance log

Per-change measurements for the optimization work, all produced by the fixed
`benchmark/full_stack_replay.sh` replay of `~/olive_ws/bags/maze_1loop` (one
maze loop, ~203 s), stack pinned to P-cores, debug topics off.

## Headline

- **Round 1** — executor swap, VO thread cap, matcher scratch reuse (apt-default
  build): total stack **46.6 → ~34 CPU-s (−26%)**.
- **Round 2** — source-built GTSAM/PCL + AVX2 unlocking the nanoflann kNN backend
  (opt-in build, see [BUILDING.md](../BUILDING.md)): **fusion 20.8 → ~17.2
  CPU-s (−17%)**, effectively all from nanoflann.
- **Guardrails held at every commit:** 65/65 unit tests, post-anchor ATE RMSE in
  the 0.07–0.09 m run-to-run band (baseline 0.080), local-stream max step
  ≤ 0.044 m, VO path ratio 0.96–0.99. Both build paths (apt-default /
  optimized) verified green.
- **Rejected with evidence:** `-march` on apt libs (GTSAM ABI crash),
  `-ffast-math` (breaks NaN/degeneracy guards), `ApproximateVoxelGrid`
  (under-downsamples 4.8×), Eigen `computeDirect`, Gauss-Newton block-view
  hoist, `OMP_NUM_THREADS=1`, tbbmalloc, whole-map memoization.

**Method:** CPU-seconds per process from `/proc/<pid>/stat` — hybrid-CPU-safe,
unlike perf-event counts that split across the P/E-core PMUs. ATE RMSE +
local-stream max step from `analyze_bag.py`. Machine: i9-14900HX (P-cores 0-15
/ E-cores 16-31), 32 threads, `powersave` governor, Ubuntu 24.04, ROS 2 Jazzy,
gcc 13.3. In later runs ambient machine load inflates the `vo`/`whycon` columns,
so **`fusion` CPU-s is the reliable cross-run metric.**

## Runs

`CPU` columns are CPU-seconds over the replay; `ATE` = post-anchor RMSE (m);
`step` = local-odom max inter-sample step (m). **Bold** marks the wins.

| id | commit | change | fusion | vo | whycon | total | ATE | step | verdict |
|----|--------|--------|-------:|-----:|-------:|------:|------:|------:|---------|
| B0 | `99dd54c` | baseline `-O2`, no `-march` | 23.0 | 19.3 | 3.7 | 46.0 | 0.080 | 0.043 | reference |
| B1 | `62c61bb` | `-O3` + LTO default | 23.3 | 19.4 | 3.9 | 46.6 | 0.072 | 0.039 | kept (free) |
| — | `cc66a73` | `-march=x86-64-v3` on apt GTSAM | — | — | — | — | — | — | ✗ ABI crash |
| P2 | `456f9d2` | C++20 + style sweep | 23.3 | 25.2 | 5.0 | 53.5 | 0.087 | 0.044 | adopted (cost-free) |
| P3 | `355d2aa`+`8f10ddc` | EventsExecutor + VO `setNumThreads(1)` | **20.8** | **9.6** | 4.1 | 34.5 | 0.075 | 0.039 | **adopted** |
| P3b | (env) | `OMP_NUM_THREADS=1` | 21.2 | 11.1 | 4.7 | 36.9 | 0.069 | 0.039 | ✗ no win |
| P4 | `c493f00`+`8a8064b` | matcher + local-map scratch hoists | **20.4** | 9.8 | 3.4 | 33.6 | 0.072 | 0.039 | adopted |
| P5 | `b7bfd52` | compile firewall | 21.3 | 9.8 | 3.8 | 34.8 | 0.084 | 0.039 | adopted |
| R1 | `781c1af` | source GTSAM 4.2 + PCL 1.15.1 + `-march` | 20.8 | 10.3 | 3.9 | 35.0 | 0.088 | 0.045 | infra (unlocks R2) |
| R2 | `38e4a6c` | `KdTreeNanoflann` backend | **17.0** | 10.2 | 3.2 | **30.3** | 0.071 | 0.039 | **adopted** |
| — | (reverted) | `ApproximateVoxelGrid` in buildLocalMap | 23.1 | — | — | 42.2 | 0.046 | 0.040 | ✗ 4.8× denser map |
| — | (reverted) | Eigen `computeDirect` (3×3 fits) | — | — | — | — | — | — | ✗ no win |
| — | (reverted) | Gauss-Newton buffer hoist | — | — | — | — | — | — | ✗ worse |
| R4 | `7e59748` | FTZ/DAZ denormal flush | 17.6 | — | — | — | 0.085 | 0.043 | kept (insurance) |
| R5 | (env) | `LD_PRELOAD=libtbbmalloc_proxy` | 17.3 | 12.4 | 6.1 | 35.9 | 0.083 | 0.040 | ✗ no win |
| R6 | `d8cd6a3` | round-2 final (optimized build) | 17.5 | 14.8 | 7.3 | 39.6 | 0.093 | 0.040 | final |

## Detail

- **B1 `-O3`+LTO** — no measurable CPU win (the hot cycles sit inside prebuilt
  PCL `.so`s that our flags don't recompile); kept because it's free and LTO
  re-inlines across the 5-TU `fusion_node` split.
- **`-march` on apt GTSAM** — AVX2 TUs corrupt the heap at the prebuilt
  SSE2-ABI GTSAM boundary; every gtsam-linked test dies with `double free`.
  `EIGEN_MAX_ALIGN_BYTES=16` does not fix it — needs source-built deps (→ R1).
- **P2** — fusion CPU identical to B1, i.e. the C++20 / style sweep is provably
  cost-free.
- **P3 (round-1 win)** — executor swap −11 % fusion; `cv::setNumThreads(1)`
  roughly halves `vo` CPU by killing OpenCV's TBB idle-spin.
- **P4** — micro-bench `align()` 3.75 → 3.50 ms (−7 %), output bit-identical;
  fusion peak RSS 90 → 82 MB from buffer reuse. (`setTarget` rebuild-skip and
  whole-map memoization were deferred — the cache-invalidation risk outweighs
  the gain.)
- **P5** — compile-only; clean rebuild 46 → 44 s. Kept for header hygiene.
- **R1** — `-march`/AVX2 across olive + GTSAM + PCL gives no CPU win *alone*
  (kdtree is pointer-chasing, voxel filter is hash/sort), but it fixes the ABI
  crash and unlocks PCL 1.15.1. Two build workarounds, both in
  [BUILDING.md](../BUILDING.md): the gold linker (bfd emitted invalid dynamic
  relocations in `pcl_sample_consensus`) and an `inline` patch for a
  `kdtree_nanoflann.h` ODR bug.
- **R2 (round-2 win)** — `KdTreeNanoflann` micro `align()` 3.51 → 1.91 ms
  (−46 %), full-stack fusion −18 %, best ATE of the band. Algorithmic beats
  flags.
- **`ApproximateVoxelGrid`** — −48 % on the filter itself, but its hash binning
  under-downsampled: the planar local map came out 4.8× denser (106,945 vs
  22,263 pts), so the matcher paid back more than the filter saved. (The denser
  map did give the best-ever ATE, 0.046 — an accuracy/CPU dial worth
  remembering.)
- **`computeDirect` / GN buffer hoist** — neither helped under AVX2 (the
  iterative eigensolver already vectorizes well; strided `topRows` block views
  defeat contiguous GEMM). Reverted.
- **R4 FTZ/DAZ** — no measured win (denormals aren't a cost on this bag) but
  kept: two lines, zero risk, insurance against denormal stalls on real-robot
  data, and unlike `-ffast-math` it preserves NaN/degeneracy semantics. The
  first run's ATE 0.100 was variance (rerun 0.085, in band).
- **R5 tbbmalloc** — fusion within noise of R2's 17.0; not adopted. (tcmalloc
  needs a sudo install; it can be A/B'd the same way later.)
- **R6** — the apt-default build (all options OFF, stock GTSAM/PCL 1.14) was
  re-verified 65/65 green: the open-source portability gate.

## Micro-benchmarks

Google Benchmark harnesses in `benchmark/micro/`, built with the package's real
compile flags. Each has a `dump` mode that prints the result in hexfloat, used
to prove a change is bit-identical before it replaces the original.

| bench | baseline | optimized | result |
|-------|---------:|----------:|--------|
| `ScanMatcher::align()` — P4 scratch reuse | 3.75 ms | 3.50 ms | −7 %, bit-identical |
| `ScanMatcher::align()` — R2 nanoflann | 3.51 ms | 1.91 ms | **−46 %** |
| `KeyframeMap::buildLocalMap()` — P4 hoists | 4.4 ms | 4.4 ms | flat (allocs removed, not the bottleneck) |

## Environment changes

Installs performed on this machine for the benchmark / verification work, kept
as a permanent record. GTSAM and PCL are built *from source* into
`~/olive_ws/deps/` (not apt-installed). Pre-existing: google-benchmark 1.8.3,
sysstat, linux-tools/perf, libnanoflann-dev 1.5.4.

| date | package | version | command | why |
|------|---------|---------|---------|-----|
| — | (none) | — | — | no system packages installed |
