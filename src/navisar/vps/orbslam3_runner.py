"""ORB-SLAM3 launcher for external mono-inertial runs."""

from __future__ import annotations

import shlex
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass
class OrbSlam3Config:
    """Configuration for launching ORB-SLAM3."""

    command: str | None = None
    bin_path: str | None = None
    vocab_path: str | None = None
    settings_path: str | None = None
    dataset_path: str | None = None
    timestamps_path: str | None = None
    extra_args: str | None = None
    cwd: str | None = None


class OrbSlam3Runner:
    """Launch ORB-SLAM3 as an external process."""

    def __init__(self, config: OrbSlam3Config) -> None:
        self.config = config
        self._process: subprocess.Popen | None = None

    def _build_command(self) -> list[str]:
        if self.config.command:
            return shlex.split(self.config.command)

        if not self.config.bin_path:
            raise ValueError("orbslam3.command or orbslam3.bin_path must be set.")
        if not self.config.vocab_path or not self.config.settings_path:
            raise ValueError("orbslam3.vocab_path and orbslam3.settings_path are required.")

        cmd = [
            self.config.bin_path,
            self.config.vocab_path,
            self.config.settings_path,
        ]
        if self.config.dataset_path:
            cmd.append(self.config.dataset_path)
        if self.config.timestamps_path:
            cmd.append(self.config.timestamps_path)
        if self.config.extra_args:
            cmd.extend(shlex.split(self.config.extra_args))
        return cmd

    def start(self) -> None:
        """Start the ORB-SLAM3 process."""
        if self._process is not None:
            return
        cmd = self._build_command()
        cwd = None
        if self.config.cwd:
            cwd = str(Path(self.config.cwd).expanduser())
        self._process = subprocess.Popen(cmd, cwd=cwd)

    def stop(self) -> None:
        """Stop the ORB-SLAM3 process."""
        if self._process is None:
            return
        self._process.terminate()
        self._process.wait(timeout=5)
        self._process = None
