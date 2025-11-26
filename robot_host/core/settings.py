# robot_host/core/settings.py

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

from ..config.pin_config import ENC0_A, ENC0_B


@dataclass
class TransportSettings:
    type: str = "tcp"          # "tcp" | "serial" | "ble"
    host: Optional[str] = None
    port: Optional[int] = None
    serial_port: Optional[str] = None
    baudrate: int = 115200 
    ble_name: Optional[str] = None


@dataclass
class FeatureSettings:
    telemetry: bool = True
    encoder: bool = False
    motion: bool = False
    modes: bool = True
    camera: bool = False
    # easy to extend later:
    # gpio: bool = False
    # pwm: bool = False
    # servo: bool = False
    # stepper: bool = False
    # dc_motor: bool = False


@dataclass
class EncoderDefaults:
    encoder_id: int = 0
    # 👇 these were `pin_a: int(ENC0_A)` which *calls* int() instead of type-hinting
    pin_a: int = ENC0_A
    pin_b: int = ENC0_B


@dataclass
class HostSettings:
    transport: TransportSettings
    features: FeatureSettings
    encoder_defaults: EncoderDefaults

    @classmethod
    def load(cls, profile: str = "default") -> "HostSettings":
        base = Path(__file__).resolve().parent.parent
        cfg_path = base / "config" / f"robot_profile_{profile}.yaml"

        data = yaml.safe_load(cfg_path.read_text())

        transport = TransportSettings(**data["transport"])
        features = FeatureSettings(**data["features"])
        encoder_defaults = EncoderDefaults(
            **data.get("encoder_defaults", {})
        )

        return cls(
            transport=transport,
            features=features,
            encoder_defaults=encoder_defaults,
        )
