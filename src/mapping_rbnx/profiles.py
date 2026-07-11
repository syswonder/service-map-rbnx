"""Named, field-validated RTAB-Map tuning profiles."""

from __future__ import annotations

from typing import Any


# These values were read back from both known-good v0.1 databases (`315` and
# `ranger_3f`) with rtabmap-info. They are not generic RTAB-Map defaults.
RTABMAP_PROFILES: dict[str, dict[str, object]] = {
    "ranger_mini_v3": {
        "RGBD/CreateOccupancyGrid": True,
        "Rtabmap/DetectionRate": 5.0,
        "RGBD/LinearUpdate": 0.05,
        "RGBD/AngularUpdate": 0.05,
        "Mem/NotLinkedNodesKept": True,
    },
}


def choose_provider_record(
    records: list[Any], provider_id: str, contract_id: str
) -> Any | None:
    """Select one Atlas record without depending on response ordering."""
    if not records:
        return None
    if not provider_id and len(records) > 1:
        candidates = sorted({str(record.provider_id) for record in records})
        raise RuntimeError(
            f"multiple Atlas providers expose {contract_id}: {candidates}; "
            "set config.sensor_providers for this sensor"
        )
    return records[0]


def resolve_occupancy_sources(
    raw_sources: object | None, available_sources: set[str]
) -> dict[str, object]:
    """Translate explicit 2D-grid policy into RTAB-Map parameters.

    Atlas discovery answers which streams exist. It must not decide which
    streams are stable enough to build a robot's occupancy grid.
    """
    if raw_sources is None:
        return {}
    if not isinstance(raw_sources, (list, tuple)) or not raw_sources:
        raise RuntimeError(
            "occupancy_sources must be a non-empty list containing lidar and/or depth"
        )
    sources = {str(value).strip().lower() for value in raw_sources}
    supported = {"lidar", "depth"}
    unknown = sources - supported
    if unknown:
        raise RuntimeError(
            f"unknown occupancy source(s) {sorted(unknown)}; options: {sorted(supported)}"
        )
    missing = sources - available_sources
    if missing:
        raise RuntimeError(
            f"occupancy source(s) {sorted(missing)} were requested but not resolved from Atlas"
        )
    grid_sensor = {
        frozenset({"lidar"}): 0,
        frozenset({"depth"}): 1,
        frozenset({"lidar", "depth"}): 2,
    }[frozenset(sources)]
    return {
        "Grid/Sensor": grid_sensor,
        "Grid/FromDepth": "depth" in sources,
    }


def select_rtabmap_inputs(
    raw_inputs: object | None, resolved: dict[str, str]
) -> dict[str, str]:
    """Apply an explicit RTAB-Map subscription policy to Atlas results."""
    if raw_inputs is None:
        return dict(resolved)
    if not isinstance(raw_inputs, (list, tuple)) or not raw_inputs:
        raise RuntimeError(
            "rtabmap_inputs must be a non-empty list containing lidar, rgbd, imu and/or odom"
        )
    inputs = {str(value).strip().lower() for value in raw_inputs}
    supported = {"lidar", "rgbd", "imu", "odom"}
    unknown = inputs - supported
    if unknown:
        raise RuntimeError(
            f"unknown RTAB-Map input(s) {sorted(unknown)}; options: {sorted(supported)}"
        )

    selected: dict[str, str] = {}
    if "lidar" in inputs:
        for key in ("scan_topic", "lidar_topic"):
            if resolved.get(key):
                selected[key] = resolved[key]
        if not ({"scan_topic", "lidar_topic"} & selected.keys()):
            raise RuntimeError("RTAB-Map lidar input was requested but not resolved from Atlas")
    if "rgbd" in inputs:
        if not resolved.get("rgb_topic") or not resolved.get("depth_topic"):
            raise RuntimeError("RTAB-Map rgbd input was requested but not resolved from Atlas")
        selected["rgb_topic"] = resolved["rgb_topic"]
        selected["depth_topic"] = resolved["depth_topic"]
    if "odom" in inputs:
        if not resolved.get("odom_topic"):
            raise RuntimeError("RTAB-Map odom input was requested but not resolved from Atlas")
        selected["odom_topic"] = resolved["odom_topic"]
    if "imu" in inputs:
        if not resolved.get("imu_topic"):
            raise RuntimeError("RTAB-Map imu input was requested but not resolved from Atlas")
        selected["imu_topic"] = resolved["imu_topic"]
    return selected


def resolve_rtabmap_overrides(
    profile_name: str, raw: object | None
) -> dict[str, object]:
    """Merge a named profile with scalar deploy overrides."""
    if profile_name and profile_name not in RTABMAP_PROFILES:
        raise RuntimeError(
            f"unknown rtabmap_profile {profile_name!r}; "
            f"options: {sorted(RTABMAP_PROFILES)}"
        )
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise RuntimeError(
            "rtabmap_params must be a mapping of parameter names to scalar values"
        )

    normalized: dict[str, object] = dict(RTABMAP_PROFILES.get(profile_name, {}))
    for key, value in raw.items():
        if not isinstance(key, str) or not key.strip():
            raise RuntimeError("rtabmap_params keys must be non-empty strings")
        if isinstance(value, (dict, list, tuple)) or value is None:
            raise RuntimeError(f"rtabmap_params[{key!r}] must be a scalar value")
        normalized[key] = value
    return normalized
