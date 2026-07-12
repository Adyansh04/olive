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
| B1 | 62c61bb | `-O3` + LTO default (**working baseline** from here) | 23.3 | 19.4 | 3.9 | 46.6 | 0.072 | 0.039 | **No measurable CPU win** (within noise vs B0) — hot cycles sit in prebuilt PCL `.so`s our flags don't recompile. Kept: free (43 s clean build), LTO re-inlines across the 5-TU split, standard practice. ATE delta vs B0 is run-to-run estimator noise. |
| — | cc66a73 | `-march=x86-64-v3` **tried and rejected** | — | — | — | — | — | — | AVX2/FMA TUs corrupt the heap at the prebuilt (SSE2-ABI) GTSAM boundary: every gtsam-linked unit test dies with `double free or corruption`; `EIGEN_MAX_ALIGN_BYTES=16` pin does NOT fix it. Would require rebuilding GTSAM (+PCL) from source with matching flags — out of scope. Negative result documented in CMakeLists + MODERNIZATION_PLAN §8. |
| P2 | 456f9d2 | Phase-2 style/C++20 sweep (C++20, std::numbers, aggregates, const& callbacks, tidy sweep, nodiscard/using-enum/designated-init) | 23.3 | 25.2 | 5.0 | 53.5 | 0.087 | 0.044 | **fusion CPU identical to B1 (23.3)** — sweep provably cost-free. vo/whycon deltas are replay-timing variance (VO processed 528 updates vs 418; its raw drift swings 7–25 m run-to-run). ATE within the 0.07–0.09 run band. 65/65 tests at every commit. |
| P3 | 355d2aa + 8f10ddc | EventsExecutor (both nodes) + `cv::setNumThreads(1)` in vo_node | **20.8** | **9.6** | 4.1 | **34.5** | 0.075 | 0.039 | **−26% total vs B1 (46.6 → 34.5)**. Attribution: fusion −2.5 s (−11%) = executor (fusion has no OpenCV); vo −10..15 s (~−50%+) = thread cap killing the TBB idle spin. VO output healthy (468 updates, ratio 0.98). Biggest win of the series so far. |
| P3b | (env test) | `OMP_NUM_THREADS=1` on the stack (PCL's internal OpenMP) | 21.2 | 11.1 | 4.7 | 36.9 | 0.069 | 0.039 | **No win** (deltas within run noise vs P3) — PCL's per-scan OpenMP isn't a measurable cost at this cloud size. NOT adopted into the launch. |
| P4 | c493f00 + 8a8064b | Hot-path hoists: matcher kNN scratch + solver reuse; local-map lazy position-tree + reserve + scratch reuse | **20.4** | 9.8 | 3.4 | **33.6** | 0.072 | 0.039 | Micro A/B: align 3.75→3.50 ms (−7%), **bit-identical dumps** both files. buildLocalMap micro flat (4.4 ms — allocs removed but not dominant); fusion **peak RSS 90→82 MB** from buffer reuse. `setTarget` rebuild-skip + whole-map memo **deferred**: hit-rate collapses while moving, invalidation surface not worth it (critique verdict). |
| P5 | b7bfd52 | Compile firewall (forward-declared components in fusion_node.hpp) + **final confirmation run** | 21.3 | 9.8 | 3.8 | 34.8 | 0.084 | 0.039 | Runtime unchanged (compile-only change, as expected). Build: clean 46→44 s, incremental ~30 s — **modest**, LTO link dominates; kept for header hygiene + fewer TU recompiles per header edit. |

| R1 | 781c1af | **Source-built GTSAM 4.2 + PCL 1.15.1, `-march=x86-64-v3`** (`OLIVE_NATIVE_OPT=ON`, BUILDING.md path) | 20.8 | 10.3 | 3.9 | 35.0 | 0.088 | 0.045 | march + AVX2/FMA across olive+GTSAM+PCL: **no measurable CPU win by itself** (kdtree is pointer-chasing, voxel is hash/sort). Its value: fixes the round-1 ABI crash (**65/65 green under march**) and unlocks PCL 1.15.1. Workarounds needed: gold linker (bfd emitted invalid dyn relocs in pcl_sample_consensus), `inline` patch for `kdtree_nanoflann.h` ODR bug — both documented in BUILDING.md. |
| R2 | 38e4a6c | **`KdTreeNanoflann` backend** (`OLIVE_USE_NANOFLANN=ON`) | **17.0** | 10.2 | 3.2 | **30.3** | 0.071 | 0.039 | Micro: align **3.51→1.91 ms (−46%)**, setTarget −19%. Full stack: **fusion −18%**. Best ATE of the run band. Biggest single fusion win of both rounds — algorithmic beats flags, as predicted. |

| — | (reverted) | `ApproximateVoxelGrid` in buildLocalMap | 23.1 | — | — | 42.2 | 0.046 | 0.040 | Micro −48% on the filter itself, **but** its hash binning under-downsampled: planar local map came out **4.8× denser** (106,945 vs 22,263 pts) → matcher paid far more than the filter saved (fusion 17.0→23.1). REJECTED/reverted. (Denser map did give the best-ever ATE 0.046 — accuracy/CPU trade worth remembering.) |
| — | (reverted) | Eigen `computeDirect` for the 3×3 feature fits | — | — | — | — | — | — | Micro: align 2.01–2.02 ms vs 1.91 baseline — **no win** under AVX2 (iterative path vectorizes well). Reverted. |
| — | (reverted) | Gauss-Newton jacobian/residual member-buffer hoist | — | — | — | — | — | — | Micro: align 2.18 ms — **worse** (strided `topRows` block views defeat contiguous GEMM). Reverted. |
| R4 | 7e59748 | FTZ/DAZ denormal flush on spin threads | 16.6/17.6 | — | — | 33.2/40.6 | 0.100→0.085 | 0.041/0.043 | **No measured CPU win** (denormals not a real cost on this bag); first-run ATE 0.100 was variance (rerun 0.085, in band). KEPT anyway: 2 lines, zero risk, insurance against denormal stalls on real-robot data; preserves NaN/degeneracy semantics unlike -ffast-math. |
| R5 | (env test) | `LD_PRELOAD=libtbbmalloc_proxy` | 17.3 | 12.4 | 6.1 | 35.9 | 0.083 | 0.040 | **No win** (fusion within noise of R2's 17.0; vo/whycon inflation is ambient load). NOT adopted. tcmalloc untested (not installed; needs sudo — can be A/B'd later the same way). |

| R6 | d8cd6a3 | Round-2 final (optimized build: source deps + march + nanoflann + FTZ; + C++20 set, tidy leftovers, config structs) | 17.5 | 14.8 | 7.3 | 39.6 | 0.093 | 0.040 | Fusion stable at **17.0–17.6** across R2/R4/R5/R6 (−16–18% vs round-1's 20.8). vo/whycon totals inflated by ambient machine load in later runs — fusion CPU is the reliable series metric. Portability gate re-verified: apt-default build (no deps, options OFF) still **65/65 green** on stock GTSAM/PCL 1.14. |

**Round-2 series result (fusion node): 20.8 → ~17.2 CPU-s (−17%)** — effectively all from the nanoflann kNN backend; the source-built/AVX2 infrastructure is what makes it possible (and safe). Evidence-based rejects this round: ApproximateVoxelGrid, `computeDirect`, GN block-view hoist, tbbmalloc. Both build paths (apt default / optimized opt-in per BUILDING.md) verified green.

**Round-1 series result: 46.6 → ~34 CPU-s (−26%)**, all from Phase 3+4 (executor swap, VO thread cap, matcher scratch reuse). Guardrails held throughout: ATE RMSE stayed in the 0.07–0.09 run-to-run band (baseline 0.080), local-stream max step ≤ 0.044 m, VO path ratio 0.96–0.99, 65/65 unit tests at every commit. Rejected with evidence: `-march=x86-64-v3` (GTSAM ABI heap corruption), `-ffast-math` (kills NaN/degeneracy guards), `OMP_NUM_THREADS=1` (no effect), whole-map memoization (hit-rate collapses in motion).

## Micro-benchmarks

(recorded per Phase-4 change; ns/iter original vs optimized, bit-identical assert)

## Environment changes

Installs performed on this machine for the benchmark/verification work, so
there is a permanent record. (Pre-existing: google-benchmark 1.8.3, sysstat,
linux-tools/perf.)

| date | package | version | command | why |
|------|---------|---------|---------|-----|
| — | (none yet) | | | |
