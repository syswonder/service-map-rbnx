#!/usr/bin/env python3
"""Static contracts for selecting CycloneDDS in the mapping container."""

from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parent


class CycloneDDSRuntimeTests(unittest.TestCase):
    def _run_start(self, enable_viz: str | None) -> tuple[list[str], list[str]]:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "package"
            fake_bin = root / "bin"
            fake_x11 = root / "x11"
            package.mkdir()
            fake_bin.mkdir()
            fake_x11.mkdir()
            docker_log = root / "docker.log"
            display_log = root / "display.log"

            (fake_bin / "docker").write_text(
                '#!/usr/bin/env bash\n'
                'printf "docker %s\\n" "$*" >> "$DOCKER_LOG"\n',
                encoding="utf-8",
            )
            (fake_bin / "rbnx").write_text(
                '#!/usr/bin/env bash\nprintf "/tmp/robonix-api\\n"\n',
                encoding="utf-8",
            )
            (fake_bin / "xset").write_text(
                '#!/usr/bin/env bash\nprintf "xset %s %s\\n" "${DISPLAY:-}" "$*" >> "$DISPLAY_LOG"\n',
                encoding="utf-8",
            )
            (fake_bin / "xhost").write_text(
                '#!/usr/bin/env bash\nprintf "xhost %s %s\\n" "${DISPLAY:-}" "$*" >> "$DISPLAY_LOG"\n',
                encoding="utf-8",
            )
            for executable in fake_bin.iterdir():
                executable.chmod(0o755)

            # Use an isolated socket directory so this test never touches the
            # host X server and remains deterministic on headless CI workers.
            source = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
            source = source.replace("/tmp/.X11-unix", str(fake_x11))
            wrapper = root / "start.sh"
            wrapper.write_text(source, encoding="utf-8")
            wrapper.chmod(0o755)

            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{fake_bin}:/usr/bin:/bin",
                    "DOCKER_LOG": str(docker_log),
                    "DISPLAY_LOG": str(display_log),
                    "RBNX_PACKAGE_ROOT": str(package),
                    "ROBONIX_MAPPING_FORCE": "docker",
                    "DISPLAY": "",
                }
            )
            if enable_viz is None:
                env.pop("MAPPING_ENABLE_VIZ", None)
            else:
                env["MAPPING_ENABLE_VIZ"] = enable_viz

            subprocess.run(
                ["bash", str(wrapper)],
                check=True,
                env=env,
                capture_output=True,
                text=True,
            )
            docker_calls = docker_log.read_text(encoding="utf-8").splitlines()
            display_calls = (
                display_log.read_text(encoding="utf-8").splitlines()
                if display_log.exists()
                else []
            )
            return docker_calls, display_calls

    def test_default_image_installs_cyclonedds_rmw(self) -> None:
        dockerfile = (ROOT / "docker" / "Dockerfile").read_text(encoding="utf-8")
        self.assertIn("ros-humble-rmw-cyclonedds-cpp", dockerfile)

    def test_start_wrapper_forwards_cyclonedds_uri(self) -> None:
        start = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        self.assertIn('-e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"', start)
        self.assertIn(
            '-e ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}"',
            start,
        )
        self.assertIn(
            '-e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}"', start
        )

    def test_visualization_defaults_off_without_x11_side_effects(self) -> None:
        docker_calls, display_calls = self._run_start(None)
        self.assertEqual(display_calls, [])
        run = next(call for call in docker_calls if call.startswith("docker run "))
        self.assertIn("MAPPING_ENABLE_VIZ=false", run)
        self.assertNotIn("DISPLAY=", run)
        self.assertNotIn("QT_X11_NO_MITSHM", run)

    def test_explicit_false_never_probes_or_authorizes_x11(self) -> None:
        docker_calls, display_calls = self._run_start("false")
        self.assertEqual(display_calls, [])
        run = next(call for call in docker_calls if call.startswith("docker run "))
        self.assertIn("MAPPING_ENABLE_VIZ=false", run)
        self.assertNotIn("DISPLAY=", run)

    def test_visualization_authorization_is_revoked_on_cleanup(self) -> None:
        docker_calls, display_calls = self._run_start("true")
        run = next(call for call in docker_calls if call.startswith("docker run "))
        self.assertIn("MAPPING_ENABLE_VIZ=true", run)
        self.assertIn("DISPLAY=:0", run)
        self.assertEqual(
            display_calls,
            ["xset :0 q", "xhost :0 +local:docker", "xhost :0 -local:docker"],
        )


if __name__ == "__main__":
    unittest.main()
