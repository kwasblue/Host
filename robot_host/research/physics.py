# robot_host/simulation/physics.py
from __future__ import annotations
import math

class DiffDrivePhysics:
    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.vx = 0.0
        self.omega = 0.0

    def set_velocity(self, vx: float, omega: float):
        self.vx = float(vx)
        self.omega = float(omega)

    def step(self, dt: float):
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        self.theta += self.omega * dt

    def imu(self):
        return {"gz_dps": math.degrees(self.omega), "az_g": 1.0}

    def encoders(self):
        ticks = int(self.x * 1000)
        return {"left": ticks, "right": ticks}
