#!/usr/bin/env zsh
# Build + run a micro-bench against the CURRENT source tree, using the exact
# compile flags of the component's real TU (mirrored from compile_commands.json)
# so numbers match the shipped build.
#
# Usage: benchmark/micro/build_and_run.sh scan_matcher|keyframe_map [dump|bench-args...]
set -e
source /opt/ros/jazzy/setup.zsh 2>/dev/null

HERE=${0:A:h}
PKG=$HERE/../..
DB=/home/adyansh/olive_ws/build/olive/compile_commands.json
OUT=/home/adyansh/olive_ws/perf_data/micro
mkdir -p "$OUT"

case "$1" in
  scan_matcher)
    COMPONENT=src/fusion/frontend/scan_matcher.cpp
    BENCH=$HERE/bench_scan_matcher.cpp
    LIBS=(-lpcl_common -lpcl_kdtree -lpcl_search -lgomp)
    ;;
  keyframe_map)
    COMPONENT=src/fusion/graph/keyframe_map.cpp
    BENCH=$HERE/bench_keyframe_map.cpp
    LIBS=(-lpcl_common -lpcl_kdtree -lpcl_search -lpcl_filters -L/opt/ros/jazzy/lib/x86_64-linux-gnu -Wl,-rpath,/opt/ros/jazzy/lib/x86_64-linux-gnu -lgtsam -lgomp)
    ;;
  *) echo "usage: $0 scan_matcher|keyframe_map [dump|bench-args]"; exit 2 ;;
esac
shift

# Flags of the component's real TU: keep -I/-isystem/-D/-std/-O/-g/-f/-m, drop
# LTO (irrelevant for the bench, slows the link) and the -o/-c/source parts.
FLAGS=$(python3 - "$DB" "$COMPONENT" <<'PY'
import json, shlex, sys
db, comp = sys.argv[1], sys.argv[2]
for e in json.load(open(db)):
    if e['file'].endswith(comp):
        toks = shlex.split(e['command'])
        keep, skip_next = [], False
        for i, t in enumerate(toks[1:]):
            if skip_next: skip_next = False; continue
            if t in ('-o', '-c'): skip_next = (t == '-o'); continue
            if t.endswith('.cpp') or t.endswith('.o'): continue
            if t.startswith('-flto') or t == '-fno-fat-lto-objects': continue
            keep.append(t)
        print(' '.join(keep)); break
PY
)
[[ -z "$FLAGS" ]] && { echo "component TU not in compile db — build the package first"; exit 1; }

NAME=$(basename "$BENCH" .cpp)
BIN=$OUT/$NAME
eval g++ $FLAGS -I"$PKG/include" -c "$PKG/$COMPONENT" -o "$OUT/${NAME}_component.o"
eval g++ $FLAGS -I"$PKG/include" -c "$BENCH" -o "$OUT/${NAME}.o"
g++ "$OUT/${NAME}.o" "$OUT/${NAME}_component.o" -o "$BIN" "${LIBS[@]}" -lbenchmark -lpthread
echo "built $BIN"
taskset -c 0-15 "$BIN" "$@"
