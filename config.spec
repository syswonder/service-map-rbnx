# service-map-rbnx provider instance configuration
#
# This file documents the YAML object passed through a robot deployment entry:
#
# service:
#   - name: mapping
#     url: https://github.com/syswonder/service-map-rbnx
#     config: <the fields below>
#
# It is a human- and model-readable contract, not a separately parsed schema.

required:
  sensor_providers:
    type: mapping[string, string]
    description: >-
      Sensor role to Atlas provider_id. A key enables that Mapping input and
      its value selects the provider. Supported roles are lidar2d, lidar3d,
      rgb, depth, imu, and odom. RGB-D requires both rgb and depth.
    example:
      lidar3d: roof_lidar
      rgb: front_camera
      depth: front_camera
      odom: base_chassis

optional:
  algo:
    type: string
    default: rtabmap
    allowed: [rtabmap, dlio, fastlio2]
    description: Mapping engine. fastlio2 is retained only for diagnostics.

  occupancy_sources:
    type: list[string]
    default: all resolved occupancy-capable inputs
    allowed_items: [lidar, depth]
    description: Inputs used to build the 2D occupancy grid.

  rtabmap_params:
    type: mapping[string, scalar]
    description: >-
      Optional final overrides on the deploy-owned params_file. Keys use
      RTAB-Map names such as Grid/FootprintLength. Values must be scalar.

  params_file:
    type: path
    path_base: directory containing robonix_manifest.yaml
    description: >-
      Deploy-owned YAML mapping of RTAB-Map parameters. Relative paths resolve
      from the directory containing robonix_manifest.yaml. Copy the upstream
      config/rtabmap_params.template.yaml into the deploy repository as a
      starting point; the upstream template is never loaded at runtime.

  deskew_lidar:
    type: boolean
    default: false
    description: Deskew a lidar3d PointCloud2. Requires per-point timestamps.

  base_frame:
    type: string
    default: base_link
  odom_frame:
    type: string
    default: odom
  use_sim_time:
    type: boolean
    default: false

  map_mode:
    type: string
    default: mapping
    allowed: [mapping, localization]
  map_id:
    type: string
    description: Saved map to load when starting in localization mode.
  reset_map:
    type: boolean
    default: false
  webui_port:
    type: integer_or_string
    default: 8091
    description: Set to 0 or an empty string to disable the Mapping web UI.

advanced_compatibility:
  rtabmap_inputs:
    type: list[string]
    allowed_items: [lidar, rgbd, imu, odom]
    description: >-
      Optional subset of resolved providers to pass into RTAB-Map. New
      deployments normally omit this and bind only the providers they use.
  sensors:
    deprecated: true
    replacement: sensor_providers
    description: Legacy boolean role table; accepted only for migration.
  rtabmap_profile:
    deprecated: true
    replacement: params_file or rtabmap_params
    description: Known legacy profiles still apply and emit a migration warning.
  platform:
    ignored: true
    description: Runtime target selection belongs to the package manifest/env.
  map_frame:
    ignored: true
    description: Mapping currently publishes the fixed map frame named map.
