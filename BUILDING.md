# Building OLIVE

## Default build (recommended start)

Uses the stock ROS 2 Jazzy apt dependencies (`ros-jazzy-gtsam`, `libpcl-dev`
1.14, `ros-jazzy-pcl-conversions`). Portable, no extra steps:

```zsh
source /opt/ros/jazzy/setup.zsh
cd ~/olive_ws
colcon build --symlink-install --packages-select olive
source install/setup.zsh
```

## Optimized build (opt-in): source-built GTSAM + PCL with AVX2/FMA

The scan pipeline's hot zones live inside GTSAM/PCL. The apt libraries are
compiled for baseline x86-64 (SSE2) — and mixing AVX2-compiled olive code with
them **corrupts the heap at the GTSAM boundary** (Eigen alignment ABI), which
is why `OLIVE_NATIVE_OPT` refuses to be flipped casually. The supported fast
path rebuilds both libraries with matching flags and unlocks:

- `-march=x86-64-v3` (AVX2/FMA) across olive + GTSAM + PCL,
- **PCL 1.15.1's `KdTreeNanoflann`** (`OLIVE_USE_NANOFLANN`) — measured ~2×
  faster correspondence search than FLANN in the scan matcher (the #1
  profiled hot zone; see `benchmark/RESULTS.md`).

Numbers are validated by ATE/continuity on the replay benchmark, not
bit-identity (FMA changes float rounding).

### 1. Fetch pinned sources

```zsh
sudo apt install libnanoflann-dev   # header-only, needed by PCL's nanoflann search
mkdir -p ~/olive_ws/deps && touch ~/olive_ws/deps/COLCON_IGNORE
vcs import ~/olive_ws/deps < ~/olive_ws/src/olive/deps.repos   # gtsam + pcl (ignore perception_pcl failure here)
vcs import ~/olive_ws/src  < ~/olive_ws/src/olive/deps.repos   # perception_pcl (ignore gtsam/pcl failure here)
touch ~/olive_ws/src/perception_pcl/pcl_ros/COLCON_IGNORE \
      ~/olive_ws/src/perception_pcl/perception_pcl/COLCON_IGNORE   # only pcl_conversions is needed
```

### 2. Build GTSAM 4.2.0

```zsh
cd ~/olive_ws/deps/gtsam && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/olive_ws/deps/gtsam-install \
  -DCMAKE_CXX_FLAGS="-march=x86-64-v3" \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_WITH_TBB=ON -DGTSAM_BUILD_UNSTABLE=ON \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_PYTHON=OFF
make -j$(nproc) install
```

`GTSAM_USE_SYSTEM_EIGEN=ON` is load-bearing: olive and GTSAM must agree on the
Eigen version and alignment ABI.

### 3. Build PCL 1.15.1 (trimmed, no VTK)

```zsh
cd ~/olive_ws/deps/pcl && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/olive_ws/deps/pcl-install \
  -DCMAKE_CXX_FLAGS="-march=x86-64-v3" \
  -DCMAKE_SHARED_LINKER_FLAGS="-fuse-ld=gold" -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=gold" \
  -DWITH_VTK=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF -DWITH_CUDA=OFF \
  -DBUILD_visualization=OFF -DBUILD_apps=OFF -DBUILD_tools=OFF -DBUILD_examples=OFF \
  -DBUILD_global_tests=OFF -DBUILD_tracking=OFF -DBUILD_people=OFF \
  -DBUILD_outofcore=OFF -DBUILD_simulation=OFF -DBUILD_stereo=OFF
make -j$(nproc) install
```

Two workarounds, both verified needed as of pcl-1.15.1 on Ubuntu 24.04:

- **`-fuse-ld=gold`**: bfd ld leaks link-time relocation types
  (`R_X86_64_REX_GOTPCRELX`/`PLT32`) into `libpcl_sample_consensus`'s dynamic
  relocations, which the runtime loader rejects (`unexpected reloc type 0x2a`).
- **`kdtree_nanoflann.h` ODR patch**: `pcl::search::internal::square_if_l2`'s
  specializations are missing `inline` (multiple-definition link errors).
  After `make install`, patch the installed header:

```zsh
sed -i 's/^float$/inline float/' \
  ~/olive_ws/deps/pcl-install/include/pcl-1.15/pcl/search/kdtree_nanoflann.h
```

### 4. Build the workspace against the source deps

```zsh
cd ~/olive_ws
colcon build --symlink-install --packages-select pcl_conversions --cmake-args \
  -DPCL_DIR=$HOME/olive_ws/deps/pcl-install/share/pcl-1.15
colcon build --symlink-install --packages-select olive --cmake-args \
  -DOLIVE_NATIVE_OPT=ON -DOLIVE_USE_NANOFLANN=ON \
  -DPCL_DIR=$HOME/olive_ws/deps/pcl-install/share/pcl-1.15 \
  -DGTSAM_DIR=$HOME/olive_ws/deps/gtsam-install/lib/cmake/GTSAM \
  -DGTSAM_UNSTABLE_DIR=$HOME/olive_ws/deps/gtsam-install/lib/cmake/GTSAM_UNSTABLE
```

### 5. Runtime library resolution

`LD_LIBRARY_PATH` must prefer the source-built libraries (it outranks the
binaries' RUNPATH; otherwise the apt SSE2 GTSAM is loaded and the process
crashes). Add after sourcing the workspace:

```zsh
export LD_LIBRARY_PATH=$HOME/olive_ws/deps/gtsam-install/lib:$HOME/olive_ws/deps/pcl-install/lib:$LD_LIBRARY_PATH
```

`benchmark/full_stack_replay.sh` does this automatically when the deps dirs
exist. Verify with `ldd install/olive/lib/olive/fusion_node | grep -E "gtsam|pcl_common"`.

### Validation

Run the unit tests and the replay benchmark (guardrails: ATE RMSE in the
0.07–0.09 band, local-step ≤ ~0.045 m, VO ratio ≥ 0.95):

```zsh
colcon test --packages-select olive && colcon test-result
benchmark/full_stack_replay.sh optimized_check
```
