"""Named, field-validated RTAB-Map tuning profiles."""

from __future__ import annotations


# These values were read back from both known-good v0.1 databases (`315` and
# `ranger_3f`) with rtabmap-info. They are not generic RTAB-Map defaults.
RTABMAP_PROFILES: dict[str, dict[str, object]] = {
    "ranger_mini_v3": {
        "Grid/Sensor": 0,
        "RGBD/CreateOccupancyGrid": True,
        "Rtabmap/DetectionRate": 5.0,
        "RGBD/LinearUpdate": 0.05,
        "RGBD/AngularUpdate": 0.05,
        "Mem/NotLinkedNodesKept": True,
    },
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
