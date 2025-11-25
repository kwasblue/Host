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
PINS_JSON = Path("/Users/kwasiaddo/projects/Host/robot_host/config/pins.json")


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
    # ----------------------------------------------------------------------
    # Robot Core
    # ----------------------------------------------------------------------
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

    # ----------------------------------------------------------------------
    # LED
    # ----------------------------------------------------------------------
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

    # ----------------------------------------------------------------------
    # GPIO
    # ----------------------------------------------------------------------
    "CMD_GPIO_WRITE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Write a digital value to a logical GPIO channel.",
        "payload": {
            "channel": {"type": "int", "required": True},
            "value": {"type": "int", "required": True},
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

    # ----------------------------------------------------------------------
    # PWM
    # ----------------------------------------------------------------------
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

    # ----------------------------------------------------------------------
    # Servo
    # ----------------------------------------------------------------------
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

    # *** UPDATED: supports `duration_ms` for servo interpolation ***
    "CMD_SERVO_SET_ANGLE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set servo angle in degrees.",
        "payload": {
            "servo_id": {"type": "int", "required": True},
            "angle_deg": {"type": "float", "required": True},
            "duration_ms": {
                "type": "int",
                "required": False,
                "default": 0,
                "description": "Interpolation duration in milliseconds (0 = immediate).",
            },
        },
    },

    # ----------------------------------------------------------------------
    # Stepper
    # ----------------------------------------------------------------------
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
                "default": 1000.0,  # steps per second
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

    "CMD_STEPPER_ENABLE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Enable or disable a stepper driver (via enable pin).",
        "payload": {
            "motor_id": {"type": "int", "required": True},
            "enable": {
                "type": "bool",
                "required": False,
                "default": True,  # True = enable, False = disable
            },
        },
    },
    
    "CMD_ULTRASONIC_ATTACH": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Attach/configure an ultrasonic sensor for the given logical sensor_id.",
        "payload": {
            # Which UltrasonicManager slot to use (0..MAX_SENSORS-1)
            "sensor_id": {
                "type": "int",
                "required": True,
                "default": 0,
            },
        },
    },

    "CMD_ULTRASONIC_READ": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Trigger a single ultrasonic distance measurement.",
        "payload": {
            # Same logical sensor_id you attached earlier
            "sensor_id": {
                "type": "int",
                "required": True,
                "default": 0,
            },
        },
    },
    
    # ----------------------------------------------------------------------
    # Telemetry control
    # ----------------------------------------------------------------------
    "CMD_TELEM_SET_INTERVAL": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set telemetry publish interval in milliseconds (0 = disable).",
        "payload": {
            "interval_ms": {
                "type": "int",
                "required": True,
                "default": 100,  # 10 Hz
            },
        },
    },

    # ----------------------------------------------------------------------
    # Logging (NEW)
    # ----------------------------------------------------------------------
    "CMD_SET_LOG_LEVEL": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set MCU logging verbosity level.",
        "payload": {
            "level": {
                "type": "string",
                "required": True,
                "enum": ["debug", "info", "warn", "error", "off"],
                "default": "info",
            },
        },
    },
    # ----------------------------------------------------------------------
    # Encoders
    # ----------------------------------------------------------------------
    "CMD_ENCODER_ATTACH": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Attach/configure a quadrature encoder with runtime pins.",
        "payload": {
            "encoder_id": {
                "type": "int",
                "required": True,
                "default": 0,
            },
            "pin_a": {
                "type": "int",
                "required": True,
                "default": 32,
            },
            "pin_b": {
                "type": "int",
                "required": True,
                "default": 33,
            },
        },
    },

    "CMD_ENCODER_READ": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Request current tick count for a given encoder.",
        "payload": {
            "encoder_id": {
                "type": "int",
                "required": True,
                "default": 0,
            },
        },
    },

    "CMD_ENCODER_RESET": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Reset the tick count for a given encoder back to zero.",
        "payload": {
            "encoder_id": {
                "type": "int",
                "required": True,
                "default": 0,
            },
        },
    },
    # ----------------------------------------------------------------------
    # DC Motor
    # ----------------------------------------------------------------------
    "CMD_DC_SET_SPEED": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set DC motor speed and direction for a given motor ID.",
        "payload": {
            "motor_id": {
                "type": "int",
                "required": True,
                "description": "Logical DC motor ID (0..3).",
            },
            "speed": {
                "type": "float",
                "required": True,
                "description": "Normalized speed in [-1.0, 1.0]; sign = direction.",
                "min": -1.0,
                "max": 1.0,
            },
        },
    },

    "CMD_DC_STOP": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Stop a DC motor (set speed to zero).",
        "payload": {
            "motor_id": {
                "type": "int",
                "required": True,
                "description": "Logical DC motor ID (0..3).",
            },
        },
    },
        # ----------------------------------------------------------------------
    # DC Motor – Velocity PID
    # ----------------------------------------------------------------------
    "CMD_DC_VEL_PID_ENABLE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Enable or disable closed-loop velocity PID control for a DC motor.",
        "payload": {
            "motor_id": {
                "type": "int",
                "required": True,
                "description": "Logical DC motor ID (0..3).",
            },
            "enable": {
                "type": "bool",
                "required": True,
                "description": "True to enable velocity PID, False to disable.",
            },
        },
    },

    "CMD_DC_SET_VEL_TARGET": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set desired angular velocity target for a DC motor's PID controller.",
        "payload": {
            "motor_id": {
                "type": "int",
                "required": True,
                "description": "Logical DC motor ID (0..3).",
            },
            "omega": {
                "type": "float",
                "required": True,
                "description": "Target angular velocity in rad/s (sign indicates direction).",
            },
        },
    },

    "CMD_DC_SET_VEL_GAINS": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Configure PID gains for DC motor velocity control.",
        "payload": {
            "motor_id": {
                "type": "int",
                "required": True,
                "description": "Logical DC motor ID (0..3).",
            },
            "kp": {
                "type": "float",
                "required": True,
                "description": "Proportional gain for velocity PID.",
            },
            "ki": {
                "type": "float",
                "required": True,
                "description": "Integral gain for velocity PID.",
            },
            "kd": {
                "type": "float",
                "required": True,
                "description": "Derivative gain for velocity PID.",
            },
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
    {
        "name": "ULTRASONIC_TRIG",
        "channel": 1,
        "pin_name": "ULTRA0_TRIG",
        "mode": "output",
    },
    {
        "name": "ULTRASONIC_ECHO",
        "channel": 2,
        "pin_name": "ULTRA0_ECHO",
        "mode": "input",
    },

    # --- DC motor direction pins (L298N Motor A) ---
    {
        "name": "MOTOR_LEFT_IN1",
        "channel": 3,
        "pin_name": "MOTOR_LEFT_IN1",
        "mode": "output",
    },
    {
        "name": "MOTOR_LEFT_IN2",
        "channel": 4,
        "pin_name": "MOTOR_LEFT_IN2",
        "mode": "output",
    },

    # --- Stepper enable (so host *can* poke EN if desired) ---
    {
        "name": "STEPPER0_EN",
        "channel": 5,
        "pin_name": "STEPPER0_EN",
        "mode": "output",
    },

    # --- Encoder pins exposed as GPIO inputs (for debug / GPIO_READ) ---
    {
        "name": "ENC0_A",
        "channel": 6,
        "pin_name": "ENC0_A",
        "mode": "input",
    },
    {
        "name": "ENC0_B",
        "channel": 7,
        "pin_name": "ENC0_B",
        "mode": "input",
    },
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
