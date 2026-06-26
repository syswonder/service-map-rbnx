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
the live map/pose off the shared rclpy node (map_ops._get_node()).

Disabled by default; set MAPPING_WEBUI_PORT (e.g. 8091) to enable. The server
binds 0.0.0.0 so it's reachable from the operator's laptop on the robot LAN.
"""
from __future__ import annotations

import io
import json
import logging
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from . import map_ops

log = logging.getLogger("mapping_rbnx.webui")

MAPS_DIR = os.environ.get("MAPPING_MAPS_DIR", "/mapping/maps")
MAP_TOPIC = os.environ.get("MAPPING_MAP_TOPIC", "/map")
POSE_TOPIC = os.environ.get("MAPPING_POSE_TOPIC", "/robonix/map/pose")

# Latest map / pose, filled by ROS subscriptions on the shared node.
_latest = {"grid": None, "pose": None}
_subscribed = False
_sub_lock = threading.Lock()


def _ensure_subscriptions() -> None:
    """Subscribe (once) to the live occupancy grid + pose so the UI can render
    them. Best-effort — if ROS isn't up the preview just stays empty."""
    global _subscribed
    with _sub_lock:
        if _subscribed:
            return
        node = map_ops._get_node()
        if node is None:
            return
        try:
            from nav_msgs.msg import OccupancyGrid
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
            latched = QoSProfile(depth=1)
            latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            latched.reliability = QoSReliabilityPolicy.RELIABLE

            def _on_grid(msg):
                _latest["grid"] = msg

            def _on_pose(msg):
                _latest["pose"] = msg

            node.create_subscription(OccupancyGrid, MAP_TOPIC, _on_grid, latched)
            node.create_subscription(PoseWithCovarianceStamped, POSE_TOPIC, _on_pose, 10)
            _subscribed = True
            log.info("webui subscribed: map=%s pose=%s", MAP_TOPIC, POSE_TOPIC)
        except Exception as e:  # noqa: BLE001
            log.warning("webui subscriptions failed: %s", e)


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
 #mapimg{background:#000;border-radius:4px;max-width:720px;cursor:crosshair}
 button{background:#2d6cdf;color:#fff;border:0;border-radius:6px;padding:7px 12px;cursor:pointer;font-size:14px}
 button.alt{background:#3a4150}
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
  <div><img id=mapimg src="/api/map.png" alt="map"></div>
  <div class=muted>click the map to set a pose estimate (relocalize)</div>
 </div>
 <div class=card style="min-width:280px">
  <h3 style="margin:4px 0 10px">Save current map</h3>
  <div style="display:flex;gap:8px">
   <input id=saveid placeholder="map_id e.g. lab_3f" style="flex:1">
   <button onclick="doSave()">Save</button>
  </div>
  <h3 style="margin:16px 0 10px">Mode</h3>
  <div style="display:flex;gap:8px">
   <button onclick="doSwitch('mapping')">Mapping (build)</button>
   <button class=alt onclick="doSwitch('localization')">Localization</button>
  </div>
  <h3 style="margin:16px 0 10px">Library</h3>
  <div id=lib class=lib></div>
 </div>
</div>
<script>
function setStatus(t){document.getElementById('status').textContent=t}
function refreshMap(){document.getElementById('mapimg').src='/api/map.png?'+Date.now()}
setInterval(refreshMap,2000)
async function poll(){try{let s=await (await fetch('/api/state')).json();
  setStatus(s.has_map?('map '+s.width+'×'+s.height+' @'+s.resolution+'m  pose='+(s.pose?('('+s.pose.x.toFixed(2)+', '+s.pose.y.toFixed(2)+', '+s.pose.theta.toFixed(2)+')'):'—')):'no map yet')}
  catch(e){setStatus('disconnected')}}
setInterval(poll,1500);poll()
async function loadLib(){let m=await (await fetch('/api/maps')).json();
 let el=document.getElementById('lib');el.innerHTML='';
 if(!m.length){el.innerHTML='<div class=muted>no saved maps yet</div>';return}
 for(const x of m){let d=document.createElement('div');d.className='mapitem';
  d.innerHTML=`<img src="/api/maps/${x.map_id}/preview.png?${Date.now()}">
   <div style="flex:1"><b>${x.map_id}</b><div class=muted>${(x.db_size/1e6).toFixed(1)} MB${x.has_db?'':' · no db'}</div></div>
   <button class=alt onclick="doLoad('${x.map_id}')">Load</button>`;
  el.appendChild(d)}}
setInterval(loadLib,5000);loadLib()
async function doSave(){let id=document.getElementById('saveid').value.trim();
 if(!id){alert('enter a map_id');return}setStatus('saving '+id+'…');
 let r=await (await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({map_id:id})})).json();
 setStatus(r.detail||'saved');loadLib()}
async function doLoad(id){if(!confirm('Load map '+id+' (localization)?'))return;setStatus('loading '+id+'…');
 let r=await (await fetch('/api/load',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({map_id:id,mode:'localization'})})).json();
 setStatus(r.detail||'loaded')}
async function doSwitch(mode){setStatus('switching to '+mode+'…');
 let r=await (await fetch('/api/switch_mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:mode})})).json();
 setStatus(r.detail||('mode '+mode))}
document.getElementById('mapimg').addEventListener('click',async ev=>{
 let img=ev.target,rect=img.getBoundingClientRect();
 let fx=(ev.clientX-rect.left)/rect.width, fy=(ev.clientY-rect.top)/rect.height;
 let s=await (await fetch('/api/state')).json();
 if(!s.has_map){alert('no map');return}
 let x=s.origin_x+fx*s.width*s.resolution;
 let y=s.origin_y+(1-fy)*s.height*s.resolution;
 if(!confirm('Seed pose estimate at ('+x.toFixed(2)+', '+y.toFixed(2)+')?'))return;
 setStatus('seeding pose…');
 let r=await (await fetch('/api/pose_estimate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:x,y:y,theta:0})})).json();
 setStatus(r.detail||'seeded')})
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
                return self._send(200, "image/png", _grid_to_png(g, _latest["pose"]))
            if p == "/api/state":
                _ensure_subscriptions()
                g = _latest["grid"]
                st = {"has_map": g is not None}
                if g is not None:
                    st.update(width=g.info.width, height=g.info.height,
                              resolution=round(g.info.resolution, 4),
                              origin_x=g.info.origin.position.x,
                              origin_y=g.info.origin.position.y)
                ps = _latest["pose"]
                if ps is not None:
                    pp = ps.pose.pose
                    q = pp.orientation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))
                    st["pose"] = {"x": pp.position.x, "y": pp.position.y, "theta": yaw}
                return self._json(st)
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
                out = map_ops.save_map_impl(body.get("map_id", ""), body.get("note", ""),
                                            active_db=_active_db_hint())
                return self._json(out)
            if p == "/api/load":
                out = map_ops.load_map_impl(
                    body.get("map_id", ""), body.get("mode", "localization"),
                    bool(body.get("has_initial_pose", False)),
                    float(body.get("x", 0.0)), float(body.get("y", 0.0)),
                    float(body.get("theta", 0.0)))
                return self._json(out)
            if p == "/api/pose_estimate":
                out = map_ops.pose_estimate_impl(
                    float(body.get("x", 0.0)), float(body.get("y", 0.0)),
                    float(body.get("theta", 0.0)))
                return self._json(out)
            if p == "/api/switch_mode":
                out = map_ops.switch_mode_impl(body.get("mode", ""))
                return self._json(out)
            return self._send(404, "text/plain", b"not found")
        except Exception as e:  # noqa: BLE001
            log.exception("webui POST %s failed", p)
            return self._json({"ok": False, "detail": str(e)}, code=500)


# Set by atlas_bridge so save_map can checkpoint the live (possibly ephemeral)
# session's db; webui itself has no session state.
_active_db_fn = None


def set_active_db_hint(fn) -> None:
    global _active_db_fn
    _active_db_fn = fn


def _active_db_hint() -> str:
    try:
        return _active_db_fn() if _active_db_fn else ""
    except Exception:  # noqa: BLE001
        return ""


_server = None


def maybe_start() -> None:
    """Start the web UI iff MAPPING_WEBUI_PORT is set. Idempotent; non-fatal."""
    global _server
    port = os.environ.get("MAPPING_WEBUI_PORT", "").strip()
    if not port or _server is not None:
        return
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", int(port)), _Handler)
    except Exception as e:  # noqa: BLE001
        log.warning("webui: cannot bind port %s: %s", port, e)
        return
    _server = srv
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    log.info("map web UI on http://0.0.0.0:%s", port)
