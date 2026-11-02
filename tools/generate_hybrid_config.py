#!/usr/bin/env python3
"""Developer helper: regenerate simulation configs after building/sourcing packages."""
from pathlib import Path
from rack_simulation.generate_config import generate as generate_configs
ROOT=Path(__file__).resolve().parents[1]


def generate():
    generate_configs(ROOT/'src/robot_bringup/config',
        assets=ROOT/'src/rack_simulation/navigation',
        hardware=ROOT/'src/robot_navigation/config/navigation_robot.yaml',
        base=ROOT/'src/robot_bringup/config')
    print('Simulation configs regenerated; hardware calibration untouched')


if __name__=='__main__':generate()
