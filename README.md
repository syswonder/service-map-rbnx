# mapping_rbnx

SLAM mapping service for [Robonix](https://github.com/syswonder/robonix). It
turns a robot's lidar / camera / odom streams into a live **2D occupancy
grid**, a **3D point cloud**, and a **SLAM-corrected pose**, published under a
fixed, engine-agnostic capability surface (`robonix/service/map/*`), and
persists named maps so a robot can re-localize across restarts.

It is a Robonix **service** package: it registers with `atlas`, discovers its
sensor inputs by capability contract (never hardcoded topics), and is brought
up by `rbnx boot`. Consumers (`scene`, `nav`) bind the contracts, not the
SLAM engine.

- Capability surface, config schema, and persistence layout: **[CAPABILITY.md](CAPABILITY.md)**.

## SLAM engines (`algo`)

| algo | use | inputs |
|---|---|---|
| `rtabmap` *(default, recommended)* | sim + real; 2D/3D/RGBD, sensor-agnostic | any of lidar2d / lidar3d / rgbd (+ odom) |
| `dlio` | real-robot 3D Livox + IMU | lidar3d + imu, needs a colcon ws at `/ws/install` |
| `fastlio2` | **broken (drift)** — repro only | — |

The launch branches on whichever sensors the deploy enabled, so the *same*
`rtabmap` config maps a webots Tiago (2D lidar + RGBD) and a MID-360 robot
(3D lidar + RGBD) without code changes.

## How to integrate it on your robot

1. **Register your sensors** as Robonix primitives under the standard
   contracts (`robonix/primitive/lidar/lidar3d`, `.../camera/depth`,
   `.../chassis/odom`, …). mapping discovers them via atlas.
2. **Pick a deployment target** and reference the matching package manifest
   from your deploy `robonix_manifest.yaml`:

   ```yaml
   service:
     - name: mapping
       url: https://github.com/syswonder/service-map-rbnx
       # manifest: package_manifest.jetson-native.yaml   # x86+docker is default
       config:
         algo: rtabmap
         sensors: { lidar3d: true, rgb: true, depth: true, odom: true, imu: true }
         rtabmap_inputs: [lidar, odom]
         occupancy_sources: [lidar]
         deskew_lidar: true    # requires per-point timestamps (MID-360 format 0)
         base_frame: base_link
         use_sim_time: false
         rtabmap_profile: ranger_mini_v3
         map_mode: mapping       # fresh runtime session; or localization
   ```
3. `rbnx build -f robonix_manifest.yaml` then `rbnx boot -f robonix_manifest.yaml`.
4. Consume the map: subscribe to `robonix/service/map/occupancy_grid` /
   `.../pointcloud` / `.../pose` (resolve them via atlas).

For Ranger Mini v3, select `occupancy_sources: [lidar]` and use the named
profile instead of copying slash-named parameters into every deploy. Together
they match the two known-good v0.1 map databases: lidar-only occupancy
(`Grid/Sensor=0`), 5 Hz detection, 0.05 m/rad node thresholds, and retained
unlinked nodes. `rtabmap_params` remains an explicit per-parameter override.

With external odometry, `deskew_lidar` compensates each PointCloud2 point in
the odom frame before SLAM. Without external odometry, it enables ICP's
constant-velocity deskewing; include `imu` in `rtabmap_inputs` to initialize
the ICP orientation from the selected IMU provider. Raw Livox gyro/accel is
first converted to an orientation estimate by `imu_filter_madgwick`.

## Deployment targets

One package, three targets (selected by the deploy `manifest:` field — see
[CAPABILITY.md](CAPABILITY.md#deployment-targets)):

| target | manifest | runtime |
|---|---|---|
| x86_64 + docker | `package_manifest.yaml` | docker (`docker/Dockerfile`) |
| arm64 Jetson + docker | `package_manifest.jetson-docker.yaml` | docker (`docker/Dockerfile.jetson`, L4T) |
| arm64 Jetson + native | `package_manifest.jetson-native.yaml` | host ROS2 (`scripts/start_native.sh`) |

Add a target by adding a `package_manifest.<target>.yaml` plus a case branch
in `scripts/build.sh` — the rest of the package is unchanged.

## Saving & re-using a map

Mapping starts with a fresh runtime database. Call `save_map(map_id)` after
coverage is complete; the named map lives under `{MAPPING_MAPS_DIR}/{map_id}/`
(default: the package's `maps/` dir, which survives container restarts):

```
maps/lab_3f/rtabmap.db  occupancy.pgm  occupancy.yaml  occupancy.png  cloud.pcd  meta.yaml
```

- **Build a map:** `map_mode: mapping`. Drive the robot around, then call
  `save_map(map_id)` to publish an immutable database and previews.
- **Re-use a map:** set `map_id` plus `map_mode: localization`. Mapping copies
  the saved db to a private runtime path and re-localizes against it; the
  **map frame is stable across restarts**, so Scene can load semantic state for
  the same id.
- **Start fresh:** `map_mode: mapping` (the default). It never writes an
  existing saved map.

> Localization-mode persistence only re-anchors correctly because the map
> frame is loaded from the saved db. Without `map_mode: localization` the
> map origin resets to the robot's boot pose each run.

## Web UI (live map + runtime map ops)

Set `MAPPING_WEBUI_PORT` (e.g. `8091`) to enable a dependency-light operator
page (stdlib `http.server` + Pillow), served on `0.0.0.0` so it's reachable
from a laptop on the robot LAN (`http://<robot-ip>:8091`). Off by default.

It runs **inside the mapping bridge process**, so its buttons call the same
`map_ops` impls the gRPC/MCP capabilities use — no extra round trip — and it
reads the live `/map` + pose straight off the bridge's rclpy node.

- **Live map canvas** — occupancy grid + robot pose, with **drag-to-pan,
  wheel-zoom, a 1 m grid, and double-click-to-fit**. Same world-centered
  view model as scene's web UI (canvas backing-store pinned to display size,
  so click coordinates are exact).
- **Save** — snapshot the live map under a `map_id` (writes
  `rtabmap.db` + `occupancy.png/pgm/yaml` + `meta.yaml`).
- **Library** — every saved map with a thumbnail; **Load** re-localizes onto
  it, **Del** removes it from disk.
- **Mode** — flip **Mapping ⇄ Localization** at runtime; a badge + button
  highlight shows the current mode.
- **Reset map** — wipe the live SLAM session and rebuild from scratch (for
  when mapping diverges). Note: the origin resets to the robot's *current*
  pose, so the rebuilt frame won't match the old map (origin drift).
- **Click the map → pose estimate** — seeds `/initialpose` so rtabmap
  re-localizes; the **activity log** records the seeded pose and, a few
  seconds later, where it converged + the distance from your estimate.

These are the same operations exposed as runtime **RPC + MCP capabilities**
(so Pilot can drive them too): `save_map`, `load_map`, `pose_estimate`,
`switch_mode` (the webui adds `reset` + `delete` on top). All work on the
running rtabmap without a redeploy — `load`/`switch_mode` call rtabmap's
runtime services and fall back to a restart with the config's `map_mode` /
`map_id` when those services aren't reachable.

> The web UI has no auth — it's a LAN debug tool. Don't expose the port to an
> untrusted network.

## Layout

```
mapping_rbnx/
├── package_manifest.yaml                 x86+docker (default)
├── package_manifest.jetson-docker.yaml   arm64 Jetson + docker
├── package_manifest.jetson-native.yaml   arm64 Jetson + host ROS2
├── CAPABILITY.md                         capability surface + config spec
├── src/mapping_rbnx/atlas_bridge.py      cap registration, sensor discovery, persistence
├── launch/rtabmap_2d.launch.py           sensor-agnostic rtabmap launch
├── scripts/
│   ├── build.sh                          per-target build
│   ├── start.sh                          native↔docker dispatch
│   ├── start_engine.sh                   in-container SLAM launcher
│   ├── start_native.sh                   host-process launcher
│   └── save_map.py                       offline map snapshot (pgm/png/pcd/meta)
└── docker/                               Dockerfile, Dockerfile.jetson, compose
```

## Troubleshooting

- **`/map` never populates** — no sensor enabled, or the wrong `sensors:`
  flags. Check the `[start_engine] rtabmap scan2d=… scan3d=…` log line.
- **`map_mode=localization` errors "no saved map"** — run a `mapping` session
  with that `map_id` first, and confirm `MAPPING_MAPS_DIR` is the same path
  (mounted) across runs.
- **Map origin drifts between runs** — you're in `mapping` mode (origin =
  boot pose). Use `localization` to re-anchor to the saved map.

License: MulanPSL-2.0
