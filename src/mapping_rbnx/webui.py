# SPDX-License-Identifier: MulanPSL-2.0
"""Lightweight map web UI for the mapping service.

A dependency-light (stdlib http.server + Pillow) page that lets an operator
*see* the live SLAM map and pose, and drive the runtime map operations:
  - live 2D occupancy + robot pose preview (auto-refresh)
  - Save  → save_map(map_id)
  - Library → list saved maps with thumbnails; Load → load_map(map_id, mode)
  - Pose Estimate → click the map to seed pose_estimate(x, y, theta)

Runs inside the mapping bridge process, so its buttons call the same
map_ops impls the gRPC/MCP capabilities use — no extra round trip. It reads
the live map/pose off its own dedicated rclpy node (see _ensure_subscriptions).

The Mapping bridge enables this on port 8091 by default; deployment config may
set ``webui_port: 0`` to disable it. The server binds 127.0.0.1 by default
because it exposes unauthenticated map operations; an authenticated deployment
may explicitly set MAPPING_WEBUI_HOST otherwise.
"""
from __future__ import annotations

import collections
import io
import json
import logging
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from . import lifecycle, map_ops

log = logging.getLogger("mapping_rbnx.webui")

MAPS_DIR = os.environ.get("MAPPING_MAPS_DIR", "/mapping/maps")
MAP_TOPIC = os.environ.get("MAPPING_MAP_TOPIC", "/map")
MAP_FRAME = os.environ.get("MAPPING_MAP_FRAME", "map")
_tf_buffer = None
POSE_TOPIC = os.environ.get("MAPPING_POSE_TOPIC", "/robonix/map/pose")

# Latest map / pose / range data, filled by ROS subscriptions on the webui's
# own node. "scan" and "cloud" hold points already projected into the map
# frame, so the page can draw them straight onto the occupancy canvas and an
# operator can see whether the live returns line up with the saved map.
_latest = {"grid": None, "pose": None, "scan": None, "cloud": None}
# Topics for the range sensors, resolved through Atlas by the bridge and
# injected with set_sensor_topics(). Empty means the deployment has no such
# capability bound, and the page simply has nothing to draw.
_sensor_topics = {"scan": "", "cloud": ""}


def set_sensor_topics(scan: str = "", cloud: str = "") -> None:
    """Bind the range-sensor topics the page overlays on the map.

    Called by atlas_bridge with whatever it resolved from Atlas for
    robonix/primitive/lidar/lidar (2-D scan) and .../lidar3d (point cloud), so
    the UI follows the deployment's capability bindings instead of hardcoding
    topic names. Must be called before the first request creates the
    subscriptions; later calls are ignored for topics already subscribed.
    """
    _sensor_topics["scan"] = scan or ""
    _sensor_topics["cloud"] = cloud or ""
    log.info("webui range overlay topics: scan=%s cloud=%s",
             _sensor_topics["scan"] or "<none>", _sensor_topics["cloud"] or "<none>")
_subscribed = False
_sub_lock = threading.Lock()
# Keep the dedicated node + executor alive for the process lifetime.
_webui_node = None
_webui_exec = None

# ── activity log ──────────────────────────────────────────────────────────────
# A small in-memory ring of timestamped action records so the operator can see,
# in the page, what each button did and how a pose_estimate converged. Surfaced
# at GET /api/log; also mirrored to the Python logger.
_LOG = collections.deque(maxlen=200)
_log_lock = threading.Lock()
# Last pose_estimate seed, so convergence can be measured against where the
# robot actually settled after relocalizing.
_seed = {"x": None, "y": None, "theta": None, "t": 0.0}


def _log_add(kind: str, msg: str) -> None:
    """Append one timestamped entry (kind ∈ save|load|switch|pose|info) to the
    UI activity log and mirror it to the service logger."""
    with _log_lock:
        _LOG.append({"t": time.time(), "kind": kind, "msg": msg})
    log.info("[webui:%s] %s", kind, msg)


def _live_pose_xytheta():
    """Current map-frame pose as (x, y, yaw) from the latest /pose, or None."""
    ps = _latest.get("pose")
    if ps is None:
        return None
    pp = ps.pose.pose
    q = pp.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return pp.position.x, pp.position.y, yaw


def _track_convergence(sx: float, sy: float, stheta: float, settle_s: float = 4.0) -> None:
    """After seeding a pose, wait settle_s for rtabmap to relocalize, then log
    where the robot actually settled and its offset from the clicked estimate
    (distance in metres + heading delta in degrees)."""
    def _check():
        time.sleep(settle_s)
        cur = _live_pose_xytheta()
        if cur is None:
            _log_add("pose", "no live pose — cannot measure convergence")
            return
        cx, cy, cyaw = cur
        dist = math.hypot(cx - sx, cy - sy)
        dth = abs((cyaw - stheta + math.pi) % (2 * math.pi) - math.pi)
        _log_add("pose", f"converged → ({cx:.2f}, {cy:.2f}, {math.degrees(cyaw):.0f}°)  "
                         f"Δ from estimate = {dist:.2f} m / {math.degrees(dth):.0f}°")
    threading.Thread(target=_check, daemon=True).start()


def _ensure_subscriptions() -> None:
    """Subscribe (once) to the live occupancy grid + pose so the UI can render
    them. Best-effort — if ROS isn't up yet the preview stays empty and we
    retry on the next request.

    Uses a DEDICATED node + executor rather than map_ops' shared node. The
    shared node's SingleThreadedExecutor is already spinning by the time the
    UI is first hit, and a subscription created on an already-spinning
    SingleThreadedExecutor is not reliably serviced — the callback never fires
    and the map stays blank even though /map is publishing. Here the
    subscriptions are created BEFORE this node's executor starts spinning, so
    they are always serviced.
    """
    global _subscribed, _webui_node, _webui_exec
    with _sub_lock:
        if _subscribed:
            return
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from nav_msgs.msg import OccupancyGrid
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

            # The bridge process initialises rclpy lazily (map_ops._get_node on
            # the first map op). The UI may be hit before any map op, so init
            # the context here if needed — mirrors map_ops' own guard. Without
            # this the UI returned before ever subscribing and stayed blank.
            if not rclpy.ok():
                rclpy.init(args=None)

            # Match rtabmap's /map publisher (RELIABLE + TRANSIENT_LOCAL) so the
            # last latched grid is delivered immediately on subscribe.
            latched = QoSProfile(depth=1)
            latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            latched.reliability = QoSReliabilityPolicy.RELIABLE

            def _on_grid(msg):
                _latest["grid"] = msg

            def _on_pose(msg):
                _latest["pose"] = msg

            node = Node("mapping_webui")
            node.create_subscription(OccupancyGrid, MAP_TOPIC, _on_grid, latched)
            node.create_subscription(PoseWithCovarianceStamped, POSE_TOPIC, _on_pose, 10)
            _subscribe_range_sensors(node)
            ex = SingleThreadedExecutor()
            ex.add_node(node)
            threading.Thread(target=ex.spin, daemon=True).start()
            _webui_node, _webui_exec = node, ex
            _subscribed = True
            log.info("webui subscribed (dedicated node): map=%s pose=%s", MAP_TOPIC, POSE_TOPIC)
        except Exception as e:  # noqa: BLE001
            log.warning("webui subscriptions failed: %s", e)


# Cap on points sent to the page. A 2-D scan is a few hundred; a 3-D cloud can
# be hundreds of thousands, which no browser will draw at 1 Hz. Subsampling is
# uniform so the shape of the return stays honest.
MAX_OVERLAY_POINTS = int(os.environ.get("MAPPING_WEBUI_MAX_POINTS", "1200"))
# Cloud points further than this above or below the sensor are dropped: the
# overlay exists to be compared against a 2-D occupancy grid, and ceiling or
# floor returns only obscure that.
CLOUD_Z_BAND_M = float(os.environ.get("MAPPING_WEBUI_CLOUD_Z_BAND", "0.35"))


def _scan_points(msg) -> dict:
    """Project a LaserScan into (x, y) pairs in the sensor's own frame.

    Range readings outside [range_min, range_max], and the inf/NaN a lidar
    emits for "no return", are dropped rather than drawn at range_max, which
    would paint a fake wall around the robot.
    """
    pts = []
    ang = msg.angle_min
    lo, hi = msg.range_min, msg.range_max
    step = max(1, math.ceil(len(msg.ranges) / MAX_OVERLAY_POINTS))
    for i, r in enumerate(msg.ranges):
        if i % step == 0 and lo <= r <= hi and math.isfinite(r):
            a = ang + i * msg.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
    return {"frame": msg.header.frame_id, "pts": pts, "t": time.time()}


def _cloud_points(msg) -> dict:
    """Project a PointCloud2 into (x, y) pairs in the sensor's own frame,
    keeping only returns within CLOUD_Z_BAND_M of the sensor plane and
    subsampling to MAX_OVERLAY_POINTS."""
    try:
        from sensor_msgs_py import point_cloud2
    except Exception:  # noqa: BLE001
        return {"frame": msg.header.frame_id, "pts": [], "t": time.time()}
    pts = []
    total = max(1, msg.width * msg.height)
    step = max(1, total // (MAX_OVERLAY_POINTS * 4))
    for i, pt in enumerate(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)):
        if i % step:
            continue
        x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
        if abs(z) <= CLOUD_Z_BAND_M:
            pts.append((x, y))
            if len(pts) >= MAX_OVERLAY_POINTS:
                break
    return {"frame": msg.header.frame_id, "pts": pts, "t": time.time()}


def _frame_to_map(frame: str):
    """Latest (tx, ty, yaw) taking `frame` into the map frame, or None.

    Looked up once per request and applied to every point, rather than
    transforming each point through tf2 — the overlay is a visual check, and a
    single planar transform is what the canvas needs anyway.
    """
    if not frame or _webui_node is None:
        return None
    if _tf_buffer is None:
        return None
    try:
        import rclpy
        tr = _tf_buffer.lookup_transform(
            MAP_FRAME, frame.lstrip("/"), rclpy.time.Time())
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw
    except Exception as e:  # noqa: BLE001
        log.debug("webui overlay: no transform %s -> %s: %s", frame, MAP_FRAME, e)
        return None


def _overlay_in_map(kind: str) -> dict:
    """Range points for `kind` ("scan"|"cloud") expressed in the map frame.

    Returns {"pts": [...], "frame": ..., "stale": bool}. An empty list with a
    named frame means the transform is not available yet, which the page shows
    rather than silently drawing nothing.
    """
    data = _latest.get(kind)
    if not data or not data.get("pts"):
        return {"pts": [], "frame": "", "stale": True}
    tf = _frame_to_map(data.get("frame", ""))
    if tf is None:
        return {"pts": [], "frame": data.get("frame", ""), "stale": True,
                "detail": "no transform to %s yet" % MAP_FRAME}
    tx, ty, yaw = tf
    c, s_ = math.cos(yaw), math.sin(yaw)
    pts = [[round(tx + x * c - y * s_, 3), round(ty + x * s_ + y * c, 3)]
           for x, y in data["pts"]]
    return {"pts": pts, "frame": data.get("frame", ""),
            "stale": (time.time() - data.get("t", 0)) > 3.0}


def _subscribe_range_sensors(node) -> None:
    """Subscribe to the range topics Atlas resolved, if any.

    Sensor data is best-effort published, so both use a SENSOR_DATA profile;
    subscribing RELIABLE to a BEST_EFFORT publisher silently receives nothing.
    A missing message type or topic disables that overlay and is logged once —
    the map preview itself must keep working either way.
    """
    from rclpy.qos import qos_profile_sensor_data

    global _tf_buffer
    if _tf_buffer is None:
        try:
            import tf2_ros
            _tf_buffer = tf2_ros.Buffer()
            # The listener's /tf subscriptions must exist before this node's
            # executor starts spinning, for the same reason the map and pose
            # subscriptions do -- see _ensure_subscriptions. Created here, not
            # lazily on the first lookup, or they are never serviced and every
            # transform lookup fails forever.
            tf2_ros.TransformListener(_tf_buffer, node, spin_thread=False)
        except Exception as e:  # noqa: BLE001
            log.warning("webui overlay: tf2 unavailable, no range overlay: %s", e)
            _tf_buffer = None

    scan_topic = _sensor_topics.get("scan") or ""
    cloud_topic = _sensor_topics.get("cloud") or ""
    if scan_topic:
        try:
            from sensor_msgs.msg import LaserScan
            node.create_subscription(
                LaserScan, scan_topic,
                lambda m: _latest.__setitem__("scan", _scan_points(m)),
                qos_profile_sensor_data)
            log.info("webui overlay: subscribed scan %s", scan_topic)
        except Exception as e:  # noqa: BLE001
            log.warning("webui overlay: scan %s unavailable: %s", scan_topic, e)
    if cloud_topic:
        try:
            from sensor_msgs.msg import PointCloud2
            node.create_subscription(
                PointCloud2, cloud_topic,
                lambda m: _latest.__setitem__("cloud", _cloud_points(m)),
                qos_profile_sensor_data)
            log.info("webui overlay: subscribed cloud %s", cloud_topic)
        except Exception as e:  # noqa: BLE001
            log.warning("webui overlay: cloud %s unavailable: %s", cloud_topic, e)


def _grid_to_png(grid, pose=None) -> bytes:
    """Render a nav_msgs/OccupancyGrid to a PNG (free=white, occ=black,
    unknown=grey), origin bottom-left, with an optional robot pose marker."""
    from PIL import Image, ImageDraw
    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    data = grid.data
    img = Image.new("RGB", (w, h), (128, 128, 128))
    px = img.load()
    for j in range(h):
        row = j * w
        for i in range(w):
            v = data[row + i]
            if v < 0:
                continue  # unknown → grey
            c = 255 - int(v * 255 / 100)  # 0→white(free), 100→black(occ)
            px[i, h - 1 - j] = (c, c, c)  # flip Y so up = +y
    if pose is not None and res > 0:
        try:
            p = pose.pose.pose
            mx = int((p.position.x - ox) / res)
            my = h - 1 - int((p.position.y - oy) / res)
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            d = ImageDraw.Draw(img)
            r = 4
            d.ellipse([mx - r, my - r, mx + r, my + r], fill=(220, 30, 30))
            d.line([mx, my, mx + int(10 * math.cos(yaw)), my - int(10 * math.sin(yaw))],
                   fill=(220, 30, 30), width=2)
        except Exception:  # noqa: BLE001
            pass
    # Upscale small grids so the page isn't a postage stamp.
    scale = max(1, min(4, 700 // max(1, w)))
    if scale > 1:
        img = img.resize((w * scale, h * scale), Image.NEAREST)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


def _list_saved_maps() -> list[dict]:
    out = []
    if not os.path.isdir(MAPS_DIR):
        return out
    for name in sorted(os.listdir(MAPS_DIR)):
        d = os.path.join(MAPS_DIR, name)
        if not os.path.isdir(d):
            continue
        db = os.path.join(d, "rtabmap.db")
        meta = {}
        mp = os.path.join(d, "meta.yaml")
        if os.path.isfile(mp):
            try:
                for line in open(mp):
                    if ":" in line:
                        k, v = line.split(":", 1)
                        meta[k.strip()] = v.strip()
            except Exception:  # noqa: BLE001
                pass
        out.append({
            "map_id": name,
            "has_db": os.path.isfile(db),
            "has_preview": os.path.isfile(os.path.join(d, "occupancy.png")),
            "db_size": os.path.getsize(db) if os.path.isfile(db) else 0,
            "updated": int(os.path.getmtime(db)) if os.path.isfile(db) else 0,
            "meta": meta,
        })
    return out


_PAGE = """<!doctype html><html><head><meta charset=utf-8>
<title>Robonix · mapping</title>
<style>
 body{font-family:system-ui,sans-serif;margin:0;background:#0f1115;color:#e6e6e6}
 header{padding:10px 16px;background:#171a21;font-weight:600}
 .wrap{display:flex;gap:16px;padding:16px;flex-wrap:wrap}
 .card{background:#171a21;border:1px solid #262b36;border-radius:8px;padding:12px}
 #mapcv{background:#0a0d12;border-radius:4px;display:block;cursor:grab;touch-action:none;width:720px;height:540px;max-width:100%}
 #mapcv:active{cursor:grabbing}
 button{background:#2d6cdf;color:#fff;border:0;border-radius:6px;padding:7px 12px;cursor:pointer;font-size:14px}
 button.alt{background:#3a4150}
 button.active{background:#1f8a44;box-shadow:0 0 0 2px #2bd66f55}
 button.del{background:#7a2d2d;padding:5px 9px}
 .switch{position:relative;display:inline-flex;background:#0f1115;border:1px solid #2a3140;border-radius:999px;padding:3px;gap:0}
 .switch .knob{position:absolute;top:3px;bottom:3px;left:3px;width:calc(50% - 3px);border-radius:999px;background:#2d6cdf;transition:transform .18s ease,background .18s ease}
 .switch.loc .knob{transform:translateX(100%);background:#1f8a44}
 .switch .swopt{position:relative;z-index:1;background:transparent;color:#9aa3b2;border-radius:999px;padding:6px 16px;font-size:13px;min-width:104px;transition:color .18s ease}
 .switch .swopt.on{color:#fff;font-weight:600}
 #busy{position:fixed;inset:0;background:#000c;display:none;align-items:center;justify-content:center;z-index:60}
 #busy.on{display:flex}
 #busybox{background:#171a21;border:1px solid #3a4150;border-radius:10px;padding:22px 26px;text-align:center;min-width:280px;max-width:480px}
 #busybox h4{margin:12px 0 6px;font-size:15px}
 #busybody{font-size:12.5px;line-height:1.55;color:#9aa3b2}
 .spin{width:26px;height:26px;margin:0 auto;border:3px solid #2a3140;border-top-color:#2d6cdf;border-radius:50%;animation:sp 0.9s linear infinite}
 @keyframes sp{to{transform:rotate(360deg)}}
 .warn{background:#3a2a12;border:1px solid #6b4a15;border-radius:6px;padding:8px 10px;font-size:12px;line-height:1.5;color:#f0d9a8}
 #modal{position:fixed;inset:0;background:#000a;display:none;align-items:center;justify-content:center;z-index:50}
 #modal.on{display:flex}
 #modalbox{background:#171a21;border:1px solid #3a4150;border-radius:10px;max-width:560px;width:calc(100% - 32px);padding:18px 20px;box-shadow:0 18px 50px #000b}
 #modalbox h4{margin:0 0 10px;font-size:16px}
 #modalbox.danger h4{color:#ffb454}
 #modalbody{font-size:13px;line-height:1.6;color:#c8cdd8;white-space:pre-wrap}
 #modalbtns{display:flex;gap:8px;justify-content:flex-end;margin-top:16px}
 input,select{background:#0f1115;color:#e6e6e6;border:1px solid #2a3140;border-radius:6px;padding:6px}
 .lib{display:flex;flex-direction:column;gap:8px;min-width:240px}
 .mapitem{display:flex;gap:8px;align-items:center;border:1px solid #262b36;border-radius:6px;padding:6px}
 .mapitem img{width:64px;height:64px;object-fit:contain;background:#000;border-radius:4px}
 .muted{color:#8b93a3;font-size:12px}
 #status{padding:6px 16px;color:#8b93a3;font-size:13px}
</style></head><body>
<header>Robonix · mapping live map</header>
<div id=status>connecting…</div>
<div class=wrap>
 <div class=card>
  <canvas id=mapcv width=720 height=540></canvas>
  <div class=muted>drag = pan · wheel = zoom · double-click = fit · green dots = live lidar returns
   · <button class=alt style="padding:2px 8px" onclick="fitView()">Fit</button></div>
 </div>
 <div class=card style="min-width:280px">
  <h3 style="margin:4px 0 10px">Save current map</h3>
  <div style="display:flex;gap:8px">
   <input id=saveid placeholder="map_id e.g. lab_3f" style="flex:1">
   <button onclick="doSave()">Save</button>
  </div>
  <h3 style="margin:16px 0 10px">Mode <span id=modebadge class=muted style="font-weight:400">mode: —</span></h3>
  <div class=switch id=modeswitch>
   <span class=knob id=modeknob></span>
   <button class=swopt id=btn-mapping onclick="doSwitch('mapping')">Mapping</button>
   <button class=swopt id=btn-localization onclick="doSwitch('localization')">Localization</button>
  </div>
  <div id=modewarn class=warn style="display:none;margin-top:8px"></div>
  <div style="margin-top:10px">
   <button id=btn-pose onclick="togglePose()">Set pose estimate</button>
   <span class=muted id=rangebadge style="margin-left:8px">lidar overlay: —</span>
  </div>
  <div style="margin-top:10px">
   <button class=del onclick="doReset()">Reset map (clear &amp; rebuild)</button>
   <span class=muted>wipes the live map; origin drifts</span>
  </div>
  <h3 style="margin:16px 0 10px">Library</h3>
  <div id=lib class=lib></div>
 </div>
 <div class=card style="min-width:340px;flex:1">
  <h3 style="margin:4px 0 10px">Activity log</h3>
  <div id=logbox style="height:360px;overflow:auto;font-family:ui-monospace,monospace;font-size:12px;line-height:1.5"></div>
 </div>
</div>
<div id=busy><div id=busybox>
 <div class=spin></div>
 <h4 id=busytitle></h4>
 <div id=busybody></div>
</div></div>
<div id=modal><div id=modalbox>
 <h4 id=modaltitle></h4>
 <div id=modalbody></div>
 <div id=modalbtns>
  <button class=alt id=modalno>Cancel</button>
  <button class=del id=modalyes>Continue</button>
 </div>
</div></div>
<script>
function setStatus(t){document.getElementById('status').textContent=t}
// ── interactive canvas map: pan (drag) / zoom (wheel) / grid / pose / click-pose
// ── scene-style world-centered canvas (proven model from scene webui) ──
// fit() pins the canvas backing-store resolution to its CSS display size,
// so pointer coords map 1:1 — this is what kept the click coords honest.
const cv=document.getElementById('mapcv'),cx=cv.getContext('2d');
function fit(){if(cv.width!=cv.clientWidth)cv.width=cv.clientWidth;if(cv.height!=cv.clientHeight)cv.height=cv.clientHeight}
window.addEventListener('resize',()=>{fit();draw()});fit();
let MI=null, mapImg=null;
let center=[0,0], pxPerM=40, userMoved=false;   // world center + zoom
function w2p(x,y){return [cv.width/2+(x-center[0])*pxPerM, cv.height/2-(y-center[1])*pxPerM]}
function p2w(sx,sy){return [center[0]+(sx-cv.width/2)/pxPerM, center[1]-(sy-cv.height/2)/pxPerM]}
function reloadMapImg(){let i=new Image();i.onload=()=>{mapImg=i;draw()};i.onerror=()=>{};i.src='/api/map.png?'+Date.now()}
function fitView(){if(!MI)return;userMoved=false;fit();
 let wM=MI.width*MI.resolution,hM=MI.height*MI.resolution;
 center=MI.pose?[MI.pose.x,MI.pose.y]:[MI.origin_x+wM/2,MI.origin_y+hM/2];
 pxPerM=Math.min(cv.width/wM,cv.height/hM)*0.9;draw()}
function draw(){fit();cx.clearRect(0,0,cv.width,cv.height);
 if(!MI){cx.fillStyle='#5a6172';cx.font='13px system-ui';cx.fillText('no map yet',16,24);return}
 if(!userMoved&&MI.pose)center=[MI.pose.x,MI.pose.y];
 // occupancy underlay — map.png is already y-flipped (row0 = world max-y),
 // so place top-left at world (origin_x, origin_y+hMeters) and grow down.
 if(mapImg&&MI.resolution>0){let wM=MI.width*MI.resolution,hM=MI.height*MI.resolution;
  let tl=w2p(MI.origin_x,MI.origin_y+hM);
  cx.imageSmoothingEnabled=false;cx.drawImage(mapImg,tl[0],tl[1],wM*pxPerM,hM*pxPerM)}
 // 1 m grid aligned to world
 cx.strokeStyle='rgba(90,130,200,0.18)';cx.lineWidth=1;
 let step=pxPerM,ox=((cv.width/2)-center[0]*pxPerM)%step,oy=((cv.height/2)+center[1]*pxPerM)%step;
 cx.beginPath();
 for(let x=ox;x<cv.width;x+=step){cx.moveTo(x,0);cx.lineTo(x,cv.height)}
 for(let y=oy;y<cv.height;y+=step){cx.moveTo(0,y);cx.lineTo(cv.width,y)}
 cx.stroke();
 // range overlay: what the robot sees right now, in map coordinates. If these
 // returns do not sit on the walls of the underlay, localization is off — which
 // is the whole reason for drawing them.
 if(RANGE.cloud&&RANGE.cloud.pts&&RANGE.cloud.pts.length){cx.fillStyle='rgba(80,180,230,0.45)';
  for(const q of RANGE.cloud.pts){let p=w2p(q[0],q[1]);cx.fillRect(p[0]-1,p[1]-1,2,2)}}
 if(RANGE.scan&&RANGE.scan.pts&&RANGE.scan.pts.length){cx.fillStyle='#39d353';
  for(const q of RANGE.scan.pts){let p=w2p(q[0],q[1]);cx.fillRect(p[0]-1.5,p[1]-1.5,3,3)}}
 // pose-estimate arrow being dragged
 if(poseDrag){let a=w2p(poseDrag.x0,poseDrag.y0),b=w2p(poseDrag.x1,poseDrag.y1);
  cx.strokeStyle='#ffb454';cx.fillStyle='#ffb454';cx.lineWidth=2;
  cx.beginPath();cx.arc(a[0],a[1],5,0,7);cx.fill();
  cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke()}
 // live pose marker
 if(MI.pose){let p=w2p(MI.pose.x,MI.pose.y),yaw=MI.pose.theta;
  cx.fillStyle='#e63b3b';cx.strokeStyle='#e63b3b';cx.lineWidth=2;
  cx.beginPath();cx.arc(p[0],p[1],5,0,7);cx.fill();
  cx.beginPath();cx.moveTo(p[0],p[1]);cx.lineTo(p[0]+18*Math.cos(yaw),p[1]-18*Math.sin(yaw));cx.stroke()}}
setInterval(reloadMapImg,2000);reloadMapImg()
// interaction — fit() makes internal==display, so (clientX-rect.left) is canvas px
function pt(e){let r=cv.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top]}
let drag=null,moved=0,POSEMODE=false,poseDrag=null;
cv.addEventListener('mousedown',e=>{
 if(POSEMODE&&MI){let w=p2w(...pt(e));poseDrag={x0:w[0],y0:w[1],x1:w[0],y1:w[1]};draw();return}
 drag=pt(e);moved=0});
window.addEventListener('mouseup',()=>{if(poseDrag){finishPose();return}drag=null});
window.addEventListener('mousemove',e=>{
 if(poseDrag){let w=p2w(...pt(e));poseDrag.x1=w[0];poseDrag.y1=w[1];draw();return}
 if(!drag)return;let p=pt(e);
 center[0]-=(p[0]-drag[0])/pxPerM;center[1]+=(p[1]-drag[1])/pxPerM;
 moved+=Math.abs(p[0]-drag[0])+Math.abs(p[1]-drag[1]);userMoved=true;drag=p;draw()});
cv.addEventListener('wheel',e=>{e.preventDefault();let p=pt(e),wp=p2w(p[0],p[1]);
 pxPerM*=e.deltaY<0?1.15:1/1.15;
 center[0]=wp[0]-(p[0]-cv.width/2)/pxPerM;center[1]=wp[1]+(p[1]-cv.height/2)/pxPerM;userMoved=true;draw()},{passive:false});
cv.addEventListener('dblclick',()=>fitView());
// Pose estimate is armed explicitly and set by dragging: press where the robot
// is, drag the way it faces, release. A heading matters as much as a position —
// seeding the right spot facing backwards makes relocalization fail the same
// way a wrong spot does.
function togglePose(){POSEMODE=!POSEMODE;poseDrag=null;
 let b=document.getElementById('btn-pose');if(b)b.classList.toggle('active',POSEMODE);
 cv.style.cursor=POSEMODE?'crosshair':'grab';
 setStatus(POSEMODE?'pose estimate armed — press where the robot is, drag the way it faces':'pose estimate cancelled');draw()}
async function finishPose(){
 let d=poseDrag;poseDrag=null;POSEMODE=false;
 let b=document.getElementById('btn-pose');if(b)b.classList.remove('active');
 cv.style.cursor='grab';
 let dx=d.x1-d.x0,dy=d.y1-d.y0;
 let far=Math.hypot(dx,dy)*pxPerM>10;
 let th=far?Math.atan2(dy,dx):(MI&&MI.pose?MI.pose.theta:0);
 draw();
 if(!await askConfirm('Seed pose estimate?',
  'Position ('+d.x0.toFixed(2)+', '+d.y0.toFixed(2)+'), heading '+(th*180/Math.PI).toFixed(0)+'°'+
  (far?'':' (kept the current heading — drag further to set one).')+'\\n\\n'+
  'RTAB-Map relocalizes from this guess by scan matching. Watch the green scan returns: '+
  'once they line up with the walls of the map, the estimate has converged.',
  {yes:'Seed pose'}))return;
 setStatus('seeding pose…');
 let r=await (await fetch('/api/pose_estimate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:d.x0,y:d.y0,theta:th})})).json();
 setStatus(r.detail||'seeded')}
async function poll(){try{let s=await (await fetch('/api/state')).json();MI=s;
  if(s.mode)CURMODE=s.mode;CURMAP=s.map_id||'';applyMode();
  setStatus(s.has_map?('map '+s.width+'×'+s.height+' @'+s.resolution+'m  pose='+(s.pose?('('+s.pose.x.toFixed(2)+', '+s.pose.y.toFixed(2)+', '+s.pose.theta.toFixed(2)+')'):'—')+(s.dist_from_seed!=null?'  Δseed='+s.dist_from_seed+'m':'')):'no map yet');draw()}
  catch(e){setStatus('disconnected')}}
setInterval(poll,1000);poll()
async function loadLib(){let m=await (await fetch('/api/maps')).json();
 let el=document.getElementById('lib');el.innerHTML='';
 if(!m.length){el.innerHTML='<div class=muted>no saved maps yet</div>';return}
 for(const x of m){let d=document.createElement('div');d.className='mapitem';
  d.innerHTML=`<img src="/api/maps/${x.map_id}/preview.png?${Date.now()}">
   <div style="flex:1"><b>${x.map_id}</b><div class=muted>${(x.db_size/1e6).toFixed(1)} MB${x.has_db?'':' · no db'}</div></div>
   <button class=alt onclick="doLoad('${x.map_id}')">Load</button>
   <button class=del onclick="doDelete('${x.map_id}')">Del</button>`;
  el.appendChild(d)}}
setInterval(loadLib,5000);loadLib()
const KCOL={save:'#5bd66f',load:'#5aa9ff',switch:'#d6a85b',pose:'#d65b9a',info:'#8b93a3'};
async function loadLog(){try{let L=await (await fetch('/api/log')).json();
 let box=document.getElementById('logbox');let atBottom=box.scrollTop+box.clientHeight>=box.scrollHeight-20;
 box.innerHTML=L.map(e=>{let t=new Date(e.t*1000).toLocaleTimeString();
  let c=KCOL[e.kind]||'#8b93a3';
  return `<div><span class=muted>${t}</span> <b style="color:${c}">${e.kind}</b> ${e.msg.replace(/</g,'&lt;')}</div>`}).join('');
 if(atBottom)box.scrollTop=box.scrollHeight}catch(e){}}
setInterval(loadLog,1500);loadLog()
async function doSave(){let id=document.getElementById('saveid').value.trim();
 if(!id){setStatus('enter a map_id first');return}
 let r=await runExclusive('Saving map \u201c'+id+'\u201d\u2026',
  'RTAB-Map is paused while its database is flushed and copied, then the occupancy preview is rendered. Do not drive the robot until this finishes.',
  async()=>(await (await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({map_id:id})})).json()));
 if(r){setStatus(r.detail||'saved');loadLib()}}
async function doLoad(id){
 if(!await askConfirm('Load map '+id+' in localization mode?',
  'This replaces the live SLAM session. Anything mapped since the last Save is discarded — '+
  'save the current map first if you still need it.',
  {danger:true,yes:'Load '+id}))return;
 let r=await runExclusive('Loading map \u201c'+id+'\u201d\u2026',
  'The saved database is copied to a runtime file, RTAB-Map switches onto it, and the map is republished. Watch the green scan afterwards to see whether it relocalized.',
  async()=>(await (await fetch('/api/load',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({map_id:id,mode:'localization'})})).json()));
 if(r)setStatus(r.detail||'loaded')}
async function doSwitch(mode){
 // Switching a running RTAB-Map from localization to mapping is the one
 // transition that can destroy work: it resumes building from wherever the
 // robot currently believes it is, so if relocalization has not converged on
 // the loaded map it opens a new session instead of extending the old one.
 if(mode=='mapping'&&CURMODE=='localization'){
  let what=CURMAP?('map "'+CURMAP+'"'):'the loaded map';
  if(!await askConfirm('Switch to mapping while localized on '+what+'?',
   'Expect '+what+' to disappear from the live view.\\n\\n'+
   'Why: entering localization started a new RTAB-Map session id. RTAB-Map only links '+
   'consecutive nodes that share a session id, so everything built after this switch '+
   'forms a separate graph component, and the published map is assembled from the '+
   'component the robot is currently in. '+what+' is still in the database — it comes '+
   'back the moment a loop closure ties the two sessions together, and the saved copy on '+
   'disk is never touched.\\n\\n'+
   'So this is only safe if RTAB-Map can relocalize where you are standing. To build a '+
   'genuinely new map, restart the service with map_mode: mapping instead.',
   {danger:true,yes:'Switch to mapping'}))return}
 let r=await runExclusive('Switching to '+mode+'\u2026',
  'Asking RTAB-Map to change mode. The map itself is not touched.',
  async()=>(await (await fetch('/api/switch_mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:mode})})).json()));
 if(r&&r.ok)CURMODE=mode;
 applyMode();if(r)setStatus(r.detail||('mode '+mode))}  // poll() re-reads the real mode a second later
async function doReset(){
 if(!await askConfirm('Clear the live map and rebuild from scratch?',
  'The new map origin becomes the current robot position, so the rebuilt map will NOT '+
  'line up with the old frame — anything recorded against the previous map goes stale. '+
  'Saved maps on disk are not affected.',
  {danger:true,yes:'Clear live map'}))return;
 let r=await runExclusive('Clearing the live map\u2026',
  'RTAB-Map is resetting and resumes mapping from the current pose. Everything recorded against the old frame is now stale.',
  async()=>(await (await fetch('/api/reset',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'})).json()));
 if(r)setStatus(r.detail||'reset')}
async function doDelete(id){
 if(!await askConfirm('Delete saved map '+id+'?',
  'The saved database and its previews are removed from disk. This cannot be undone.',
  {danger:true,yes:'Delete '+id}))return;
 let r=await runExclusive('Deleting \u201c'+id+'\u201d\u2026',
  'Removing the saved database and its previews from disk.',
  async()=>(await (await fetch('/api/delete',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({map_id:id})})).json()));
 if(r){setStatus(r.detail||'deleted');loadLib()}}
let CURMODE=null,CURMAP='',RANGE={},BUSY=false;
// Save and load take tens of seconds on a real map: rtabmap is paused, the
// database is flushed and copied, previews are rendered. A second operation
// issued in the middle of that acts on a half-published map, so the page holds
// everything back behind one overlay until the first one answers.
function busyOn(title, body){BUSY=true;
 document.getElementById('busytitle').textContent=title;
 document.getElementById('busybody').textContent=body||'';
 document.getElementById('busy').classList.add('on')}
function busyOff(){BUSY=false;document.getElementById('busy').classList.remove('on')}
async function runExclusive(title, body, fn){
 if(BUSY)return null;
 busyOn(title, body);
 try{return await fn()}
 catch(e){setStatus('failed: '+e);return null}
 finally{busyOff()}}
async function pollRange(){try{let r=await (await fetch('/api/range')).json();RANGE=r;
 let el=document.getElementById('rangebadge');if(!el)return;
 let bits=[];
 for(const k of ['scan','cloud']){if(!r[k])continue;
  let n=r[k].pts?r[k].pts.length:0;bits.push(k+' '+(n?n+' pts':(r[k].detail||'waiting')))}
 el.textContent=bits.length?bits.join(' · '):'no lidar capability bound';
 draw()}catch(e){}}
setInterval(pollRange,500);pollRange()
let MODALRESOLVE=null;
function askConfirm(title, body, opts){
 opts=opts||{};
 let m=document.getElementById('modal');
 document.getElementById('modaltitle').textContent=title;
 document.getElementById('modalbody').textContent=body;
 document.getElementById('modalyes').textContent=opts.yes||'Continue';
 document.getElementById('modalbox').className=opts.danger?'danger':'';
 m.classList.add('on');
 return new Promise(res=>{MODALRESOLVE=res})}
function closeModal(v){
 document.getElementById('modal').classList.remove('on');
 if(MODALRESOLVE){let r=MODALRESOLVE;MODALRESOLVE=null;r(v)}}
document.getElementById('modalyes').onclick=()=>closeModal(true);
document.getElementById('modalno').onclick=()=>closeModal(false);
document.getElementById('modal').onclick=e=>{if(e.target.id=='modal')closeModal(false)};
document.addEventListener('keydown',e=>{if(e.key=='Escape'&&MODALRESOLVE)closeModal(false)});
function applyMode(){let mp=document.getElementById('btn-mapping'),lo=document.getElementById('btn-localization');
 let bdg=document.getElementById('modebadge');if(bdg)bdg.textContent=CURMODE?('mode: '+CURMODE):'mode: —';
 let sw=document.getElementById('modeswitch');
 if(sw)sw.classList.toggle('loc',CURMODE=='localization');
 if(mp&&lo){mp.classList.toggle('on',CURMODE=='mapping');lo.classList.toggle('on',CURMODE=='localization')}
 let w=document.getElementById('modewarn');if(!w)return;
 if(CURMODE=='localization'){w.style.display='';
  w.textContent='Localized on '+(CURMAP?('map "'+CURMAP+'"'):'a loaded map')+'. Switching to '+
   'mapping starts a separate RTAB-Map session, so this map leaves the live view until a loop '+
   'closure links the two — it is not deleted, and the saved copy is untouched. Restart with '+
   'map_mode: mapping to build a new map instead.'}
 else{w.style.display='none';w.textContent=''}}
</script></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # quiet
        return

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, "application/json", json.dumps(obj).encode())

    def do_GET(self):
        p = urlparse(self.path).path
        try:
            if p == "/" or p == "/index.html":
                return self._send(200, "text/html; charset=utf-8", _PAGE.encode())
            if p == "/api/map.png":
                _ensure_subscriptions()
                g = _latest["grid"]
                if g is None:
                    return self._send(503, "text/plain", b"no map yet")
                # Pure occupancy only — the live pose marker is drawn by the
                # canvas on top, so don't bake a second (scaled) one into the PNG.
                return self._send(200, "image/png", _grid_to_png(g))
            if p == "/api/state":
                _ensure_subscriptions()
                g = _latest["grid"]
                # Read the mode from map_ops rather than remembering one
                # here: a mode set by config, MCP or gRPC must show up in this
                # page too, and a load or reset changes it without the page
                # being involved at all.
                live = lifecycle.current()
                st = {"has_map": g is not None,
                      "mode": map_ops.get_mode_impl().get("mode", ""),
                      "map_id": live.get("map_id", "")}
                if g is not None:
                    st.update(width=g.info.width, height=g.info.height,
                              resolution=round(g.info.resolution, 4),
                              origin_x=g.info.origin.position.x,
                              origin_y=g.info.origin.position.y)
                cur = _live_pose_xytheta()
                if cur is not None:
                    st["pose"] = {"x": cur[0], "y": cur[1], "theta": cur[2]}
                    if _seed["x"] is not None:
                        st["seed"] = {"x": _seed["x"], "y": _seed["y"], "theta": _seed["theta"]}
                        st["dist_from_seed"] = round(math.hypot(cur[0] - _seed["x"],
                                                                cur[1] - _seed["y"]), 3)
                return self._json(st)
            if p == "/api/range":
                # One request for every bound overlay: the page draws them
                # together and a second round trip would let them drift apart
                # on screen. A sensor the deployment has not bound is absent
                # from the payload rather than present and empty, so the page
                # has nothing to report about a capability that is not there.
                out = {"bound": {k: v for k, v in _sensor_topics.items() if v}}
                for kind in ("scan", "cloud"):
                    if _sensor_topics.get(kind):
                        out[kind] = _overlay_in_map(kind)
                return self._json(out)
            if p == "/api/log":
                with _log_lock:
                    return self._json(list(_LOG))
            if p == "/api/maps":
                return self._json(_list_saved_maps())
            if p.startswith("/api/maps/") and p.endswith("/preview.png"):
                mid = p[len("/api/maps/"):-len("/preview.png")]
                fp = os.path.join(MAPS_DIR, map_ops._sanitize_map_id(mid), "occupancy.png")
                if os.path.isfile(fp):
                    return self._send(200, "image/png", open(fp, "rb").read())
                return self._send(404, "text/plain", b"no preview")
            return self._send(404, "text/plain", b"not found")
        except Exception as e:  # noqa: BLE001
            log.exception("webui GET %s failed", p)
            return self._send(500, "text/plain", str(e).encode())

    def do_POST(self):
        p = urlparse(self.path).path
        try:
            n = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(n) or b"{}")
            if p == "/api/save":
                mid = body.get("map_id", "")
                out = map_ops.save_map_impl(mid, body.get("note", ""))
                _log_add("save", out.get("detail") or (f"saved {mid}" if out.get("ok") else "save failed"))
                return self._json(out)
            if p == "/api/load":
                mid, mode = body.get("map_id", ""), body.get("mode", "localization")
                out = map_ops.load_map_impl(
                    mid, mode, bool(body.get("has_initial_pose", False)),
                    float(body.get("x", 0.0)), float(body.get("y", 0.0)),
                    float(body.get("theta", 0.0)))
                _log_add("load", f"{'✓' if out.get('ok') else '✗'} load {mid} ({mode}): {out.get('detail','')}")
                return self._json(out)
            if p == "/api/delete":
                mid = body.get("map_id", "")
                out = map_ops.delete_map_impl(mid)
                _log_add("delete", out.get("detail") or (f"deleted {mid}" if out.get("ok") else "delete failed"))
                return self._json(out)
            if p == "/api/reset":
                out = map_ops.reset_map_impl()
                _log_add("reset", out.get("detail") or ("map cleared" if out.get("ok") else "reset failed"))
                return self._json(out)
            if p == "/api/pose_estimate":
                x, y, th = (float(body.get("x", 0.0)), float(body.get("y", 0.0)),
                            float(body.get("theta", 0.0)))
                out = map_ops.pose_estimate_impl(x, y, th)
                if out.get("ok"):
                    _seed.update(x=x, y=y, theta=th, t=time.time())
                    _log_add("pose", f"estimate seeded → ({x:.2f}, {y:.2f}, {math.degrees(th):.0f}°); "
                                     "waiting for relocalization…")
                    _track_convergence(x, y, th)
                else:
                    _log_add("pose", f"✗ pose estimate: {out.get('detail','')}")
                return self._json(out)
            if p == "/api/switch_mode":
                mode = body.get("mode", "")
                out = map_ops.switch_mode_impl(mode)
                _log_add("switch", f"{'✓' if out.get('ok') else '✗'} switch to {mode}: {out.get('detail','')}")
                return self._json(out)
            return self._send(404, "text/plain", b"not found")
        except Exception as e:  # noqa: BLE001
            log.exception("webui POST %s failed", p)
            return self._json({"ok": False, "detail": str(e)}, code=500)




_server = None


def maybe_start() -> None:
    """Start the web UI iff MAPPING_WEBUI_PORT is set. Idempotent; non-fatal."""
    global _server
    port = os.environ.get("MAPPING_WEBUI_PORT", "").strip()
    if not port or _server is not None:
        return
    # This is an unauthenticated admin plane (delete/reset/load/pose seed), so
    # fail safe on loopback. A deployment that has an authenticated overlay
    # network may opt into another host explicitly through `webui_host`.
    host = os.environ.get("MAPPING_WEBUI_HOST", "127.0.0.1").strip() or "127.0.0.1"
    try:
        srv = ThreadingHTTPServer((host, int(port)), _Handler)
    except Exception as e:  # noqa: BLE001
        log.warning("webui: cannot bind %s:%s: %s", host, port, e)
        return
    _server = srv
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    log.info("map web UI on http://%s:%s", host, port)
