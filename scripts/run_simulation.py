#!/usr/bin/env python3
"""Build, launch Gazebo, and run one corrected-GP7 pick-and-place task."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import shlex
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PROJECTS = ROOT.parent
ROS_SETUP = Path('/opt/ros/jazzy/setup.bash')
INSTALL_SETUP = PROJECTS / '.ros2_install/setup.bash'


def sourced(command: str) -> list[str]:
    return ['bash', '-lc', f'source {shlex.quote(str(ROS_SETUP))} && source {shlex.quote(str(INSTALL_SETUP))} && {command}']


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-build', action='store_true', help='reuse the existing ROS2 build')
    parser.add_argument('--results', type=Path, default=ROOT / 'results/latest_run.json')
    args = parser.parse_args()
    if not ROS_SETUP.exists():
        parser.error('ROS 2 Jazzy not found at /opt/ros/jazzy')

    if not args.no_build:
        build_command = shlex.join([
            'colcon', 'build', '--base-paths', str(ROOT),
            '--build-base', str(PROJECTS / '.ros2_build'),
            '--install-base', str(PROJECTS / '.ros2_install'),
            '--symlink-install', '--cmake-args',
            '-DPython3_EXECUTABLE=/usr/bin/python3'])
        build = subprocess.run(sourced(build_command), cwd=PROJECTS)
        if build.returncode:
            return build.returncode
    if not INSTALL_SETUP.exists():
        parser.error(f'ROS2 install missing: {INSTALL_SETUP}; run without --no-build')

    launch = subprocess.Popen(sourced('exec ros2 launch robo2 gazebo.launch.py'),
                              cwd=PROJECTS, start_new_session=True)
    print('Gazebo starting; waiting for controller manager...', flush=True)
    try:
        ready = False
        for _ in range(120):
            if launch.poll() is not None:
                raise RuntimeError(f'Gazebo launch exited with status {launch.returncode}')
            check = subprocess.run(sourced('ros2 service list'), cwd=PROJECTS,
                                   capture_output=True, text=True)
            if '/controller_manager/list_controllers' in check.stdout:
                ready = True
                break
            time.sleep(1)
        if not ready:
            raise TimeoutError('Controller manager did not start within 120 seconds')
        print('Controllers ready; running pick-and-place.', flush=True)
        command = sourced(f'ros2 run robo2 pick_place.py --results {args.results.resolve()}')
        return subprocess.run(command, cwd=PROJECTS).returncode
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, TimeoutError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                launch.wait(timeout=5)


if __name__ == '__main__':
    raise SystemExit(main())
