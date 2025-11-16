# platform_schema.py
"""
Single source of truth for the robot platform:

- PINS: numeric mapping (from pins.json)
- COMMANDS: JSON command schema
- GPIO_CHANNELS: logical GPIO channel mapping
"""

from pathlib import Path
import json

ROOT = Path(__file__).resolve().parent

# Location of pins.json (same as in your current gen_pins.py)
PINS_JSON = Path("/Users/kwasiaddo/projects/Host/robot_host/pins.json")


# === 1) PINS: loaded from pins.json ===

def _load_pins(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("pins.json must be a JSON object {NAME: number, ...}")
    for name, value in data.items():
        if not isinstance(name, str):
            raise ValueError(f"Pin name {name!r} is not a string")
        if not isinstance(value, int):
            raise ValueError(f"Pin {name} value {value!r} is not an int")
        if not (0 <= value <= 39):
            raise ValueError(f"Pin {name} value {value} looks invalid for ESP32 GPIO")
    return data


PINS: dict[str, int] = _load_pins(PINS_JSON)


# === 2) COMMANDS: your existing schema, unchanged in spirit ===

COMMANDS: dict[str, dict] = {
    "CMD_SET_MODE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set the high-level robot mode (e.g., IDLE, ARMED, ACTIVE).",
        "payload": {
            "mode": {
                "type": "string",
                "required": True,
                "enum": ["IDLE", "ARMED", "ACTIVE", "CALIB"],
            }
        },
    },

    "CMD_SET_VEL": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set linear and angular velocity in robot frame.",
        "payload": {
            "vx": {"type": "float", "required": True, "units": "m/s"},
            "omega": {"type": "float", "required": True, "units": "rad/s"},
            "frame": {
                "type": "string",
                "required": False,
                "default": "robot",
                "enum": ["robot", "world"],
            },
        },
    },

    "CMD_STOP": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Stop all motion (soft stop).",
        "payload": {},
    },

    "CMD_ESTOP": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Emergency stop, immediately disable motion.",
        "payload": {},
    },

    "CMD_CLEAR_ESTOP": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Clear ESTOP and typically return to IDLE mode.",
        "payload": {},
    },

    "CMD_LED_ON": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Turn status LED on.",
        "payload": {},
    },

    "CMD_LED_OFF": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Turn status LED off.",
        "payload": {},
    },

    "CMD_GPIO_WRITE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Write a digital value to a logical GPIO channel.",
        "payload": {
            "channel": {"type": "int", "required": True},
            "value": {"type": "int", "required": True},  # 0 or 1
        },
    },

    "CMD_GPIO_READ": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Read a digital value from a logical GPIO channel.",
        "payload": {
            "channel": {"type": "int", "required": True},
        },
    },

    "CMD_GPIO_TOGGLE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Toggle a logical GPIO channel.",
        "payload": {
            "channel": {"type": "int", "required": True},
        },
    },

    "CMD_GPIO_REGISTER_CHANNEL": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Register or re-map a logical GPIO channel to a physical pin.",
        "payload": {
            "channel": {"type": "int", "required": True},
            "pin": {"type": "int", "required": True},
            "mode": {
                "type": "string",
                "required": False,
                "default": "output",
                "enum": ["output", "input", "input_pullup"],
            },
        },
    },

    "CMD_PWM_SET": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set PWM duty cycle for a logical channel.",
        "payload": {
            "channel": {"type": "int", "required": True},
            "duty": {"type": "float", "required": True},
            "freq_hz": {"type": "float", "required": False},
        },
    },

    "CMD_SERVO_ATTACH": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Attach a servo ID to a physical pin.",
        "payload": {
            "servo_id": {"type": "int", "required": True},
            "channel": {"type": "int", "required": True},
            "min_us": {"type": "int", "required": False, "default": 1000},
            "max_us": {"type": "int", "required": False, "default": 2000},
        },
    },

    "CMD_SERVO_DETACH": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Detach a servo ID.",
        "payload": {
            "servo_id": {"type": "int", "required": True},
        },
    },

    "CMD_SERVO_SET_ANGLE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set servo angle in degrees.",
        "payload": {
            "servo_id": {"type": "int", "required": True},
            "angle_deg": {"type": "float", "required": True},
        },
    },

    "CMD_STEPPER_MOVE_REL": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Move a stepper a relative number of steps.",
        "payload": {
            "motor_id": {"type": "int", "required": True},
            "steps": {"type": "int", "required": True},
            "speed_steps_s": {
                "type": "float",
                "required": False,
                "default": 1000.0,
            },
        },
    },

    "CMD_STEPPER_STOP": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Immediately stop a stepper motor.",
        "payload": {
            "motor_id": {"type": "int", "required": True},
        },
    },
}


# === 3) GPIO logical channels ===
#
# pin_name must be a key in PINS.

GPIO_CHANNELS: list[dict] = [
    {
        "name": "LED_STATUS",
        "channel": 0,
        "pin_name": "LED_STATUS",
        "mode": "output",
    },
    # later:
    # {
    #     "name": "ULTRASONIC_TRIG",
    #     "channel": 1,
    #     "pin_name": "ULTRASONIC_TRIG",
    #     "mode": "output",
    # },
    # {
    #     "name": "ULTRASONIC_ECHO",
    #     "channel": 2,
    #     "pin_name": "ULTRASONIC_ECHO",
    #     "mode": "input",
    # },
]


def validate_schema() -> None:
    # 1) validate GPIO_CHANNELS pin_name exists in PINS
    seen_channels = set()
    seen_names = set()

    for entry in GPIO_CHANNELS:
        name = entry["name"]
        ch = entry["channel"]
        pin_name = entry["pin_name"]
        mode = entry["mode"]

        if pin_name not in PINS:
            raise ValueError(f"GPIO_CHANNEL {name}: pin_name '{pin_name}' not in PINS")

        if mode not in ("output", "input", "input_pullup"):
            raise ValueError(
                f"{name}: mode must be 'output', 'input', or 'input_pullup'"
            )

        if ch in seen_channels:
            raise ValueError(f"Duplicate GPIO channel id {ch}")
        if name in seen_names:
            raise ValueError(f"Duplicate GPIO name {name}")

        seen_channels.add(ch)
        seen_names.add(name)

    # you can also optionally run your COMMANDS validation here if you want


# Run basic validation at import time (optional but nice)
validate_schema()
