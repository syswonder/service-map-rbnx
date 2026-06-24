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
       url: https://github.com/enkerewpo/mapping_rbnx
       # manifest: package_manifest.jetson-native.yaml   # x86+docker is default
       config:
         algo: rtabmap
         sensors: { lidar3d: true, rgbd: true, odom: true, imu: true }
         base_frame: base_link
         use_sim_time: false
         map_id: lab_3f          # optional; enables persistence
         map_mode: mapping       # or: localization
   ```
3. `rbnx build -f robonix_manifest.yaml` then `rbnx boot -f robonix_manifest.yaml`.
4. Consume the map: subscribe to `robonix/service/map/occupancy_grid` /
   `.../pointcloud` / `.../pose` (resolve them via atlas).

There is no robot-specific code to edit — sensors come from atlas, frames and
SLAM mode come from config.

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

Set `map_id` to persist. A named map lives under `{MAPPING_MAPS_DIR}/{map_id}/`
(default: the package's `maps/` dir, which survives container restarts):

```
maps/lab_3f/rtabmap.db  occupancy.pgm  occupancy.yaml  occupancy.png  cloud.pcd  meta.yaml
```

- **Build a map:** `map_mode: mapping`. Drive the robot around; the db is
  written live, and on shutdown the offline-previewable artifacts
  (pgm/png/pcd/meta) are exported. Open `occupancy.png` to eyeball it without
  rtabmap's database viewer.
- **Re-use a map:** `map_mode: localization`. mapping loads the saved db and
  re-localizes against it; the **map frame is stable across restarts**, so a
  `scene` configured with the same `map_id` re-anchors its semantic objects
  correctly. (`scene`'s object store keys on `map_id` — set it consistently.)
- **Start fresh:** `map_mode: mapping` + `reset_map: true`.

> Localization-mode persistence only re-anchors correctly because the map
> frame is loaded from the saved db. Without `map_mode: localization` the
> map origin resets to the robot's boot pose each run.

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
