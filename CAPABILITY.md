# mapping_rbnx — capability surface

SLAM mapping service. Turns wheeled/legged-robot sensor streams into a live
2D occupancy grid, a 3D point cloud, and a SLAM-corrected pose, all under a
fixed, algorithm-agnostic contract surface. Persists named maps so a robot
can re-localize against a previously-built map across restarts.

The active SLAM engine is config-selectable (`algo`); consumers never see
which engine runs — they only bind the `robonix/service/map/*` contracts.

## Provides (declared on atlas)

| contract | transport | payload | notes |
|---|---|---|---|
| `robonix/service/map/driver` | grpc | Driver lifecycle | INIT/SHUTDOWN |
| `robonix/service/map/occupancy_grid` | ros2 topic | `nav_msgs/OccupancyGrid` | 2D grid (lidar + depth fusion) |
| `robonix/service/map/pointcloud` | ros2 topic | `sensor_msgs/PointCloud2` | 3D fused cloud |
| `robonix/service/map/pose` | ros2 topic | `geometry_msgs/PoseWithCovarianceStamped` | SLAM-corrected, map frame; can jump on loop closure |
| `robonix/service/map/odom` | ros2 topic | `nav_msgs/Odometry` | SLAM-corrected, odom frame; continuous between closures |

Every `algo` backs this **same** surface (an adapter node is spawned when an
engine can't natively publish a contract), so `scene` / `nav` are
engine-agnostic.

## Consumes (via atlas discovery)

Resolved at `Driver(CMD_INIT)` from the `sensors:` config — only the enabled
ones are looked up. Topic names are **never** hardcoded; they come from
whichever primitive registered the contract.

| config flag | contract | role |
|---|---|---|
| `lidar3d` | `robonix/primitive/lidar/lidar3d` | 3D point cloud (e.g. MID-360) |
| `lidar2d` | `robonix/primitive/lidar/lidar` | 2D LaserScan |
| `imu` | `robonix/primitive/imu/imu` | IMU (lio engines) |
| `rgbd` | `robonix/primitive/camera/depth` | depth image |
| `rgb` | `robonix/primitive/camera/rgb` | rgb image |
| `odom` | `robonix/primitive/chassis/odom` | wheel/external odom (else rtabmap runs its own icp_odometry) |

If a sensor isn't on atlas yet when INIT lands, the launch disables that
subscription rather than blocking.

## Config (`config:` block → `Driver(CMD_INIT, config_json)`)

```yaml
config:
  algo: rtabmap                       # rtabmap | dlio | fastlio2[broken]
  sensors: { lidar2d: true, rgbd: true, odom: true }   # what the robot has
  # frames / time (optional; defaults = webots tiago)
  base_frame: base_link
  odom_frame: odom
  use_sim_time: true
  # map persistence (optional; rtabmap only)
  map_id: lab_3f                      # stable id; bind the SAME id in scene
  map_mode: mapping                   # mapping | localization
  reset_map: false                    # true = start a named map fresh
```

`sensors:` is required and must list at least one sensor — the package
refuses to guess.

## Map persistence (`map_id`)

A named map is stored under `{MAPPING_MAPS_DIR}/{map_id}/`:

```
maps/lab_3f/
  rtabmap.db       SLAM graph (relocalization + continued mapping)
  occupancy.pgm    nav2 map_server image (0=occ, 254=free, 205=unknown)
  occupancy.yaml   nav2 map_server metadata
  occupancy.png    same image, double-click to preview offline
  cloud.pcd        fused 3D cloud
  meta.yaml        map_id, saved_at, frame_id, resolution, size, origin
```

- **`map_mode: mapping`** builds/extends the map. The db is written live at
  `database_path`; on shutdown the previewable artifacts (pgm/png/pcd/meta)
  are exported next to it.
- **`map_mode: localization`** loads the saved db read-only and re-localizes
  against it, so the **map frame origin is stable across restarts** — the
  property `scene`'s per-`map_id` semantic store needs to re-anchor objects.
- Omit `map_id` → ephemeral (db wiped each boot; legacy behaviour).

**Binding contract:** set the SAME `map_id` here and in the `scene` service.
mapping owns the spatial map (db + map frame); scene keys its semantic
objects to that id.

## Deployment targets

Selected by the deploy entry's `manifest:` field (see the package manifests):

| target | manifest | build / start |
|---|---|---|
| x86_64 + docker | `package_manifest.yaml` (default) | `docker/Dockerfile` / docker run |
| arm64 Jetson + docker | `package_manifest.jetson-docker.yaml` | `docker/Dockerfile.jetson` / docker run |
| arm64 Jetson + native | `package_manifest.jetson-native.yaml` | host ROS2 + apt rtabmap / `start_native.sh` |

## What this does NOT do (deliberately)

- **No path planning / navigation** — that's `nav` (simple_nav / nav2_wrapper),
  which consumes this map.
- **No semantic labels** — that's `scene`; this is the spatial substrate.
- **No multi-map merge / global relocalization across maps** — one `map_id`
  per session.
- **`fastlio2` is broken (drift)** — kept reachable for repro only; use
  `rtabmap` (or `dlio` for a 3D Livox real-robot path).
