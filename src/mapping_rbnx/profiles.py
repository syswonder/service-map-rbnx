"""Named, field-validated RTAB-Map tuning profiles."""

from __future__ import annotations


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
