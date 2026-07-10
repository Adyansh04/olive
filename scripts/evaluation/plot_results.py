#!/usr/bin/env python3
"""Generate README result figures + a stats summary from the 3-loop bag."""
import math
import os
import shutil
import subprocess
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = os.environ.get("OLIVE_BAG", os.path.expanduser("~/olive_ws/bags/maze_square_3loops"))
OUT = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "media")
MARKERS = [(7.5, 7.5), (-7.5, 7.5), (-7.5, -7.5), (7.5, -7.5)]
os.makedirs(OUT, exist_ok=True)

# consistent, colour-blind-friendly palette on a white ground
C_GT, C_FUSED, C_LOCAL, C_WHEEL, C_MARK, C_CORR = (
    "#111111", "#1f77b4", "#2ca02c", "#d62728", "#9467bd", "#ff7f0e")
plt.rcParams.update({"figure.dpi": 130, "font.size": 10, "axes.grid": True,
                     "grid.alpha": 0.25, "axes.axisbelow": True})


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def open_reader(bag):
    meta = yaml.safe_load(open(os.path.join(bag, "metadata.yaml")))["rosbag2_bagfile_information"]
    tmp, uri = None, bag
    if str(meta.get("compression_mode", "")).upper() == "FILE":
        tmp = tempfile.mkdtemp(prefix="results_")
        for rel in meta["relative_file_paths"]:
            b = os.path.basename(rel)
            subprocess.run(["zstd", "-d", "-q", os.path.join(bag, rel),
                            "-o", os.path.join(tmp, b[:-5])], check=True)
        t = open(os.path.join(bag, "metadata.yaml")).read()
        t = t.replace("compression_format: zstd", "compression_format: ''").replace(
            "compression_mode: FILE", "compression_mode: ''").replace(".mcap.zstd", ".mcap")
        open(os.path.join(tmp, "metadata.yaml"), "w").write(t)
        uri = tmp
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=meta["storage_identifier"]),
           rosbag2_py.ConverterOptions("", ""))
    return r, tmp


def nearest(seq, t):
    lo, hi = 0, len(seq) - 1
    while lo < hi:
        m = (lo + hi) // 2
        if seq[m][0] < t:
            lo = m + 1
        else:
            hi = m
    if lo > 0 and abs(seq[lo - 1][0] - t) < abs(seq[lo][0] - t):
        lo -= 1
    return seq[lo]


# ---------------------------------------------------------------- read the bag
reader, tmp = open_reader(BAG)
try:
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    gt, fused, local, wheel, m2o, bias, vel, vo = [], [], [], [], [], [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in types:
            continue
        if topic == "/tf":
            m = deserialize_message(data, get_message(types[topic]))
            for tr in m.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    m2o.append((st(tr.header), tr.transform.translation.x,
                                tr.transform.translation.y, yaw_of(tr.transform.rotation)))
            continue
        m = deserialize_message(data, get_message(types[topic]))
        if topic == "/ground_truth":
            gt.append((st(m.header), m.pose.pose.position.x, m.pose.pose.position.y))
        elif topic == "/olive/odometry":
            fused.append((st(m.header), m.pose.pose.position.x, m.pose.pose.position.y))
        elif topic == "/olive/odometry_local":
            local.append((st(m.header), m.pose.pose.position.x, m.pose.pose.position.y))
        elif topic == "/odom":
            wheel.append((st(m.header), m.pose.pose.position.x, m.pose.pose.position.y))
        elif topic == "/olive/debug/bias":
            bias.append((st(m.header), m.accel.linear.x, m.accel.linear.y, m.accel.linear.z,
                         m.accel.angular.x, m.accel.angular.y, m.accel.angular.z))
        elif topic == "/olive/debug/velocity":
            vel.append((st(m.header), m.vector.x, m.vector.y, m.vector.z))
        elif topic == "/olive/visual_odom":
            vo.append((st(m.header), m.pose.pose.position.x, m.pose.pose.position.y))
finally:
    if tmp:
        shutil.rmtree(tmp, ignore_errors=True)

for s in (gt, fused, local, wheel, m2o, bias, vel, vo):
    s.sort()
t0 = fused[0][0]

# fused-vs-gt error series + anchor time
gi = 0
err = []
for (t, xf, yf) in fused:
    while gi + 1 < len(gt) and abs(gt[gi + 1][0] - t) <= abs(gt[gi][0] - t):
        gi += 1
    err.append((t - t0, math.hypot(xf - gt[gi][1], yf - gt[gi][2])))
anchor_t = next((t for t, e in err if e < 0.5), None)
if anchor_t is None:  # looser world (textured maze anchors to ~1 m, not cm)
    anchor_t = next((t for t, e in err if e < 2.0), None)
if anchor_t is None:  # never anchored at all — keep the whole run
    anchor_t = 0.0
post = [(t, e) for t, e in err if t >= anchor_t] or err
rmse = math.sqrt(sum(e * e for _, e in post) / len(post))
emax = max(e for _, e in post)

# ---------------------------------------------------------------- Fig 1: trajectory (map)
fig, ax = plt.subplots(figsize=(6.4, 6.2))
gx = [x for _, x, _ in gt]
gy = [y for _, _, y in gt]
fx = [x for t, x, _ in fused if (t - t0) >= (anchor_t or 0)]
fy = [y for t, _, y in fused if (t - t0) >= (anchor_t or 0)]
ax.plot(gx, gy, color=C_GT, lw=3.2, alpha=0.85, label="ground truth")
ax.plot(fx, fy, color=C_FUSED, lw=1.4, label="OLIVE fused  /olive/odometry")
ax.scatter([x for x, _ in MARKERS], [y for _, y in MARKERS], s=180, marker="s",
           c=C_MARK, edgecolors="white", zorder=5, label="fiducial markers")
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title(f"Fused trajectory vs ground truth (maze, 3 loops)\n"
             f"post-anchor ATE {rmse*100:.1f} cm RMSE, {emax*100:.1f} cm max", fontsize=11)
ax.legend(loc="upper left", fontsize=9)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "trajectory.png"))
plt.close(fig)

# ---------------------------------------------------------------- Fig 2: smooth local vs raw wheel (odom frame)
# /odom (Gazebo diff-drive) and /olive/odometry_local (OLIVE) are independent odom
# estimates with different frame origins, so centre each trajectory on its centroid
# to overlay the SHAPES: the fused local stream is a tight repeatable square; raw
# wheel dead-reckoning drifts/rotates around it.
def centre(series):
    mx = sum(r[1] for r in series) / len(series)
    my = sum(r[2] for r in series) / len(series)
    return [r[1] - mx for r in series], [r[2] - my for r in series]

fig, ax = plt.subplots(figsize=(6.4, 6.2))
wxr, wyr = centre(wheel)
lxr, lyr = centre(local)
ax.plot(wxr, wyr, color=C_WHEEL, lw=1.6, label="raw wheel  /odom  (dead-reckoning)")
ax.plot(lxr, lyr, color=C_LOCAL, lw=1.6, label="OLIVE local  /olive/odometry_local")
ax.scatter([wxr[-1]], [wyr[-1]], s=60, c=C_WHEEL, zorder=5, edgecolors="white")
ax.scatter([lxr[-1]], [lyr[-1]], s=60, c=C_LOCAL, zorder=5, edgecolors="white")
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("Odometry in the continuous odom frame\n"
             "the fused local stream stays tight; raw wheel odometry drifts away",
             fontsize=11)
ax.legend(loc="upper left", fontsize=9)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "local_vs_wheel.png"))
plt.close(fig)

# ---------------------------------------------------------------- Fig 3: error over time (log)
fig, ax = plt.subplots(figsize=(7.6, 3.6))
ax.semilogy([t for t, _ in err], [max(e, 1e-3) for _, e in err], color=C_FUSED, lw=1.2)
if anchor_t is not None:
    ax.axvline(anchor_t, color=C_CORR, ls="--", lw=1.4)
    ax.annotate(f"first marker anchor\n(spawn frame → world, ~8.5 m snap)",
                xy=(anchor_t, 0.3), xytext=(anchor_t + 25, 1.5),
                fontsize=8.5, color=C_CORR,
                arrowprops=dict(arrowstyle="->", color=C_CORR))
ax.axhline(rmse, color=C_GT, ls=":", lw=1.0)
ax.text(err[-1][0] * 0.62, rmse * 1.25, f"post-anchor RMSE {rmse*100:.1f} cm",
        fontsize=8.5, color=C_GT)
ax.set_xlabel("time [s]")
ax.set_ylabel("|fused − ground truth| [m]")
ax.set_title("Absolute position error over the run (drift-free after the first anchor)",
             fontsize=11)
ax.grid(True, which="both", alpha=0.25)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "error_time.png"))
plt.close(fig)

# ---------------------------------------------------------------- Fig 4: continuity (local step vs map->odom step)
def steps(series, use_xy=(1, 2)):
    out = []
    for a, b in zip(series, series[1:]):
        out.append((b[0] - t0, math.hypot(b[use_xy[0]] - a[use_xy[0]],
                                          b[use_xy[1]] - a[use_xy[1]])))
    return out

local_steps = steps(local)
m2o_steps = steps(m2o)
fig, ax = plt.subplots(figsize=(7.6, 3.6))
ax.semilogy([t for t, _ in m2o_steps], [max(s, 1e-4) for _, s in m2o_steps],
            color=C_CORR, lw=1.0, alpha=0.9, label="map→odom TF step (the correction)")
ax.semilogy([t for t, _ in local_steps], [max(s, 1e-4) for _, s in local_steps],
            color=C_LOCAL, lw=1.0, label="local odom step (Nav2's input)")
ax.axhline(0.05, color=C_GT, ls=":", lw=1.0)
lmax = max(s for _, s in local_steps)
mmax = max(s for _, s in m2o_steps)
ax.set_xlabel("time [s]")
ax.set_ylabel("inter-sample step [m]")
ax.set_title(f"REP-105 split: corrections jump map→odom (max {mmax:.2f} m), "
             f"never the local stream (max {lmax*100:.1f} cm)", fontsize=10.5)
ax.legend(loc="center right", fontsize=8.5)
ax.grid(True, which="both", alpha=0.25)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "continuity.png"))
plt.close(fig)

# ---------------------------------------------------------------- Fig 5: online IMU bias + velocity
fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.6, 5.0), sharex=True)
bt = [b[0] - t0 for b in bias]
a1.plot(bt, [b[6] for b in bias], color=C_FUSED, lw=1.4, label="gyro bias z (yaw)")
a1.plot(bt, [b[4] for b in bias], color=C_LOCAL, lw=1.0, alpha=0.7, label="gyro bias x")
a1.plot(bt, [b[5] for b in bias], color=C_WHEEL, lw=1.0, alpha=0.7, label="gyro bias y")
a1.set_ylabel("gyro bias [rad/s]")
a1.set_title("Online IMU bias & velocity estimated by the tight-coupled graph", fontsize=11)
a1.legend(loc="upper right", fontsize=8, ncol=3)
vt = [v[0] - t0 for v in vel]
a2.plot(vt, [v[1] for v in vel], color=C_FUSED, lw=1.2, label="vx")
a2.plot(vt, [v[2] for v in vel], color=C_LOCAL, lw=1.2, label="vy")
a2.set_ylabel("velocity [m/s]")
a2.set_xlabel("time [s]")
a2.legend(loc="upper right", fontsize=8, ncol=2)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "imu_state.png"))
plt.close(fig)

# ---------------------------------------------------------------- Fig 6: monocular VO trajectory
# VO (vo_odom frame), wheel (odom) and ground truth (map) each start at the same
# physical pose but in different frames, so shift each to its own start point and
# overlay the shapes. VO is wheel-scaled + planar -> expect the square shape to be
# tracked with bounded drift (it is the weakest modality, not marker-anchored).
vo_drift = None
if vo:
    def rel(series):
        x0, y0 = series[0][1], series[0][2]
        return [x - x0 for _, x, _ in series], [y - y0 for _, _, y in series]

    fig, ax = plt.subplots(figsize=(6.4, 6.2))
    gxr, gyr = rel(gt)
    wxr, wyr = rel(wheel)
    vxr, vyr = rel(vo)
    ax.plot(gxr, gyr, color=C_GT, lw=2.6, alpha=0.8, label="ground truth")
    ax.plot(wxr, wyr, color=C_WHEEL, lw=1.3, alpha=0.8, label="raw wheel  /odom")
    ax.plot(vxr, vyr, color=C_MARK, lw=1.5, label="monocular VO  /olive/visual_odom")
    ax.scatter([vxr[-1]], [vyr[-1]], s=60, c=C_MARK, zorder=5, edgecolors="white")
    ax.set_aspect("equal")
    ax.set_xlabel("x from start [m]")
    ax.set_ylabel("y from start [m]")
    # final drift of VO vs ground truth, both start-aligned
    vo_drift = math.hypot(vxr[-1] - gxr[-1], vyr[-1] - gyr[-1])
    ax.set_title("Monocular visual odometry vs ground truth (start-aligned)\n"
                 f"{len(vo)} VO updates, wheel-scaled; final drift {vo_drift:.2f} m",
                 fontsize=11)
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "vo_trajectory.png"))
    plt.close(fig)

# ---------------------------------------------------------------- summary
print("=== RESULT SUMMARY ===")
print(f"duration: {err[-1][0]:.0f} s   fused samples: {len(fused)}   local samples: {len(local)}")
print(f"first anchor at: {anchor_t:.1f} s   (pre-anchor spawn offset {err[0][1]:.2f} m)")
print(f"post-anchor ATE: RMSE {rmse*100:.1f} cm   max {emax*100:.1f} cm   final {post[-1][1]*100:.1f} cm")
print(f"local-odom max step: {lmax*100:.1f} cm    map->odom max step: {mmax:.2f} m")
print(f"raw wheel /odom endpoint (odom frame): ({wheel[-1][1]:+.1f}, {wheel[-1][2]:+.1f}) m")
print(f"local /odom endpoint (odom frame):     ({local[-1][1]:+.1f}, {local[-1][2]:+.1f}) m")
gz = [b[6] for b in bias]
print(f"gyro-bias-z range: [{min(gz):+.4f}, {max(gz):+.4f}] rad/s")
if vo:
    print(f"VO updates: {len(vo)}   start-aligned final drift vs GT: {vo_drift:.2f} m")
else:
    print("VO: no /olive/visual_odom in bag")
print("wrote:", ", ".join(sorted(os.listdir(OUT))))
