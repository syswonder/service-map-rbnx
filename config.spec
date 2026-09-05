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
    allowed: [rtabmap, slam_toolbox, dlio, fastlio2]
    description: >-
      Mapping engine. rtabmap is sensor-agnostic (2D/3D/RGB-D) and the default;
      slam_toolbox is 2D-lidar-only, CPU-only, and serializes its pose graph to
      two files instead of a database. fastlio2 is retained only for diagnostics.

  localizer:
    type: string
    default: none
    allowed: [none, amcl, beluga]
    description: >-
      Localization engine used by load_map on a saved map. `none` keeps the SLAM
      engine's own localization (needs a pose seed close enough to converge).
      `amcl` (nav2) and `beluga` (beluga_amcl, interface-compatible) run a
      particle filter over the saved occupancy grid and can relocalize with no
      prior pose at all — load_map without an initial pose then triggers global
      localization instead of guessing. CPU only, a few tens of MB.

  localizer_particles:
    type: mapping[string, scalar]
    description: >-
      Optional {min, max} particle counts for the localizer (defaults 500/2000).
      More particles converge from worse guesses and cost proportionally more.

  slam_toolbox_params:
    type: mapping[string, scalar]
    description: >-
      Scan-matching overrides for algo: slam_toolbox, the counterpart of
      rtabmap_params. Keys: min_travel_m and min_heading_rad (how far the robot
      moves before a new pose-graph node), scan_buffer (how many recent scans
      are matched against each other) and loop_search_m (radius searched for a
      loop closure). Defaults suit a slow indoor platform in a room-scale map:
      0.1 m, 0.1 rad, 30 scans, 2.5 m. Unset keys keep those defaults.

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
    description: >-
      Robot body frame used by RTAB-Map and sensor transforms. It must match
      the complete robot URDF/TF tree and the frame used by Navigation.
  odom_frame:
    type: string
    default: odom
    description: >-
      Local continuous-motion frame used for odometry and map-to-odom
      estimation. The selected odom provider must publish poses in this frame.
  navigation_odom_bridge:
    type: boolean
    default: false
    description: >-
      Opt-in split-odometry mode for robots whose accurate mapping odometry is
      too latent for Navigation. Internal RTAB-Map odometry becomes a
      message-only private trajectory and a bridge publishes map-to-navigation
      odom using the chassis pose. The default preserves legacy TF behaviour.
  navigation_odom_topic:
    type: string
    default: /odom
    description: >-
      Chassis Odometry topic used by the optional navigation bridge. It is not
      passed into RTAB-Map and does not require a sensor_providers.odom binding.
  navigation_odom_frame:
    type: string
    default: odom
    description: >-
      Frame owned by the chassis navigation odometry. When the bridge is
      enabled this must differ from the private RTAB-Map odom_frame.
  use_sim_time:
    type: boolean
    default: false
    description: >-
      Use the ROS /clock source instead of wall time. Enable this for a
      simulator only when every sensor, TF publisher, and consumer uses the
      same simulated clock.

  map_mode:
    type: string
    default: mapping
    allowed: [mapping, localization]
    description: >-
      Startup mode. mapping always starts a fresh mutable runtime database;
      localization copies the immutable artifact selected by map_id into a
      runtime database and localizes against it.
  map_id:
    type: string
    description: >-
      Saved spatial-map identifier to load when map_mode is localization. It
      is required in localization mode. New maps are named by the save_map
      operation rather than by this startup field.
  reset_map:
    type: boolean
    default: false
    description: >-
      Legacy startup reset request. New mapping sessions are already created
      with a fresh runtime database, so new deployments should omit it.
      reset_map=true is rejected in localization mode.
  webui_port:
    type: integer_or_string
    default: 8091
    description: Set to 0 or an empty string to disable the Mapping web UI.
  webui_host:
    type: string
    default: 127.0.0.1
    description: >-
      Bind address for the unauthenticated Mapping web UI. Keep the loopback
      default unless an authenticated deployment overlay protects access.

advanced_compatibility:
  rtabmap_inputs:
    type: list[string]
    allowed_items: [lidar, rgbd, imu, odom]
    description: >-
      Optional subset of resolved providers to pass into RTAB-Map. New
      deployments normally omit this and bind only the providers they use.
  sensors:
    type: mapping[string, boolean]
    deprecated: true
    replacement: sensor_providers
    description: Legacy boolean role table; accepted only for migration.
  rtabmap_profile:
    type: string
    deprecated: true
    replacement: params_file or rtabmap_params
    description: Known legacy profiles still apply and emit a migration warning.
  platform:
    type: string
    ignored: true
    description: Runtime target selection belongs to the package manifest/env.
  map_frame:
    type: string
    ignored: true
    description: Mapping currently publishes the fixed map frame named map.
