#!/usr/bin/env python3
"""Static contracts for selecting CycloneDDS in the mapping container."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parent


class CycloneDDSRuntimeTests(unittest.TestCase):
    def test_default_image_installs_cyclonedds_rmw(self) -> None:
        dockerfile = (ROOT / "docker" / "Dockerfile").read_text(encoding="utf-8")
        self.assertIn("ros-humble-rmw-cyclonedds-cpp", dockerfile)

    def test_start_wrapper_forwards_cyclonedds_uri(self) -> None:
        start = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        self.assertIn('-e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"', start)


if __name__ == "__main__":
    unittest.main()
