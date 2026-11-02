"""Compatibility module; install rack_simulation for Gazebo execution."""
import importlib
import sys
sys.modules[__name__] = importlib.import_module("rack_simulation.framework_execution")
