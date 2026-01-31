# robot_host/research/physics.py
"""
Backward-compatible physics module.

For enhanced simulation, use simulation.py directly:
    from robot_host.research.simulation import DiffDriveRobot, DiffDriveConfig
"""
from .simulation import DiffDrivePhysics

__all__ = ["DiffDrivePhysics"]
