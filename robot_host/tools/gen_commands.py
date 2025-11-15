#!/usr/bin/env python3
"""
Generate command schema artifacts from a single Python definition.

Outputs:
  - commands.json          (schema catalog)
  - CommandDefs.h          (C++ header for MCU)
  - command_defs.py        (Python constants for Host)

Run:
    python gen_commands.py
"""

from pathlib import Path
import json

# === Adjust these paths for your actual layout ===
ROOT = Path(__file__).resolve().parent

# Where to write the JSON schema catalog
JSON_OUT = ROOT / "commands.json"

# Where to write the C++ header (MCU project)
# TODO: change this to your real firmware path
CPP_OUT = Path("/Users/kwasiaddo/projects/PlatformIO/Projects/ESP32 MCU Host/include/config/CommandDefs.h")

# Where to write the Python module (Host project)
# TODO: change this to your real host path
PY_OUT = ROOT / "Host/robot_host/command_defs.py"


# === Single source of truth: all commands here ===
COMMANDS = {
    "CMD_SET_MODE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set the high-level robot mode (e.g., IDLE, ARMED, ACTIVE).",
        "payload": {
            "mode": {
                "type": "string",
                "required": True,
                "enum": ["IDLE", "ARMED", "ACTIVE", "CALIB"]
            }
        }
    },

    "CMD_SET_VEL": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set linear and angular velocity in robot frame.",
        "payload": {
            "vx": {
                "type": "float",
                "required": True,
                "units": "m/s"
            },
            "omega": {
                "type": "float",
                "required": True,
                "units": "rad/s"
            },
            "frame": {
                "type": "string",
                "required": False,
                "default": "robot",
                "enum": ["robot", "world"]
            }
        }
    },

    "CMD_STOP": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Stop all motion (soft stop).",
        "payload": {}
    },

    "CMD_ESTOP": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Emergency stop, immediately disable motion.",
        "payload": {}
    },

    "CMD_CLEAR_ESTOP": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Clear ESTOP and typically return to IDLE mode.",
        "payload": {}
    },

    "CMD_LED_ON": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Turn status LED on.",
        "payload": {}
    },

    "CMD_LED_OFF": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Turn status LED off.",
        "payload": {}
    },

        "CMD_GPIO_WRITE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Write a digital value to a logical GPIO channel.",
        "payload": {
            "channel": {  # logical ID, not raw pin
                "type": "int",
                "required": True
            },
            "value": {    # 0 or 1
                "type": "int",
                "required": True
            }
        }
    },

    "CMD_PWM_SET": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set PWM duty cycle for a logical channel.",
        "payload": {
            "channel": {
                "type": "int",
                "required": True
            },
            "duty": {      # 0.0–1.0
                "type": "float",
                "required": True
            },
            "freq_hz": {   # optional override
                "type": "float",
                "required": False
            }
        }
    },

    "CMD_SERVO_ATTACH": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Attach a servo ID to a physical pin.",
        "payload": {
            "servo_id": {
                "type": "int",
                "required": True
            },
            "channel": {   # logical PWM / pin channel
                "type": "int",
                "required": True
            },
            "min_us": {
                "type": "int",
                "required": False,
                "default": 1000
            },
            "max_us": {
                "type": "int",
                "required": False,
                "default": 2000
            }
        }
    },

    "CMD_SERVO_DETACH": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Detach a servo ID.",
        "payload": {
            "servo_id": {
                "type": "int",
                "required": True
            }
        }
    },

    "CMD_SERVO_SET_ANGLE": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Set servo angle in degrees.",
        "payload": {
            "servo_id": {
                "type": "int",
                "required": True
            },
            "angle_deg": {
                "type": "float",
                "required": True
            }
        }
    },

    "CMD_STEPPER_MOVE_REL": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Move a stepper a relative number of steps.",
        "payload": {
            "motor_id": {
                "type": "int",
                "required": True
            },
            "steps": {        # positive = one direction, negative = other
                "type": "int",
                "required": True
            },
            "speed_steps_s": {
                "type": "float",
                "required": False,
                "default": 1000.0
            }
        }
    },

    "CMD_STEPPER_STOP": {
        "kind": "cmd",
        "direction": "host->mcu",
        "description": "Immediately stop a stepper motor.",
        "payload": {
            "motor_id": {
                "type": "int",
                "required": True
            }
        }
    },


    # Add new commands here as you expand the system
    # "CMD_IMU_CALIBRATE": { ... },
    # "CMD_CFG_SET_PARAM": { ... },
}


# === Validation ===

def validate_commands(cmds: dict) -> None:
    """Basic sanity checks so the schema stays consistent."""
    for name, spec in cmds.items():
        if not name.startswith("CMD_"):
            raise ValueError(f"Command name '{name}' must start with 'CMD_'")

        kind = spec.get("kind")
        if kind not in ("cmd", "telemetry", "event", "resp", "error"):
            raise ValueError(f"{name}: invalid kind '{kind}'")

        direction = spec.get("direction")
        if direction not in ("host->mcu", "mcu->host", "both"):
            raise ValueError(
                f"{name}: direction must be 'host->mcu', 'mcu->host', or 'both'"
            )

        payload = spec.get("payload")
        if not isinstance(payload, dict):
            raise ValueError(f"{name}: payload must be an object (dict)")

        # Payload fields: just basic structure checks
        for field_name, field_spec in payload.items():
            if not isinstance(field_spec, dict):
                raise ValueError(f"{name}.{field_name}: field spec must be an object")
            if "type" not in field_spec:
                raise ValueError(f"{name}.{field_name}: field must define 'type'")


# === Generators ===

def generate_json(commands: dict) -> str:
    schema = {
        "schema_version": 1,
        "commands": commands,
    }
    return json.dumps(schema, indent=2, sort_keys=True)


def generate_cpp_header(commands: dict) -> str:
    """
    Generate a C++ header defining:
      - enum class CmdType { ... }
      - cmdTypeFromString(const std::string&)
      - cmdTypeToString(CmdType)
    """
    names = sorted(commands.keys())  # deterministic order

    # enum entries (strip "CMD_" prefix)
    enum_entries = [name.replace("CMD_", "") for name in names]

    lines: list[str] = []
    lines.append("// AUTO-GENERATED FILE — DO NOT EDIT BY HAND")
    lines.append("// Generated from COMMANDS in gen_commands.py\n")
    lines.append("#pragma once")
    lines.append("#include <string>\n")
    lines.append("enum class CmdType {")
    for entry in enum_entries:
        lines.append(f"    {entry},")
    lines.append("    UNKNOWN")
    lines.append("};\n")

    # cmdTypeFromString
    lines.append("inline CmdType cmdTypeFromString(const std::string& s) {")
    for name, entry in zip(names, enum_entries):
        lines.append(f'    if (s == "{name}") return CmdType::{entry};')
    lines.append("    return CmdType::UNKNOWN;")
    lines.append("}\n")

    # cmdTypeToString
    lines.append("inline const char* cmdTypeToString(CmdType c) {")
    lines.append("    switch (c) {")
    for name, entry in zip(names, enum_entries):
        lines.append(f'        case CmdType::{entry}: return "{name}";')
    lines.append('        default: return "UNKNOWN";')
    lines.append("    }")
    lines.append("}\n")

    return "\n".join(lines)


def generate_py_module(commands: dict) -> str:
    """
    Generate a Python module with string constants:
      CMD_SET_MODE = "CMD_SET_MODE"
      ...
      ALL_COMMANDS = [...]
    """
    names = sorted(commands.keys())

    lines: list[str] = []
    lines.append("# AUTO-GENERATED FILE — DO NOT EDIT BY HAND")
    lines.append("# Generated from COMMANDS in gen_commands.py\n")

    for name in names:
        lines.append(f'{name} = "{name}"')

    lines.append("\nALL_COMMANDS = [")
    for name in names:
        lines.append(f"    {name},")
    lines.append("]\n")

    return "\n".join(lines)


def main():
    print("[gen_commands] Validating COMMANDS...")
    validate_commands(COMMANDS)

    print(f"[gen_commands] Generating JSON: {JSON_OUT}")
    json_text = generate_json(COMMANDS)

    print(f"[gen_commands] Generating C++ header: {CPP_OUT}")
    cpp_code = generate_cpp_header(COMMANDS)

    print(f"[gen_commands] Generating Python module: {PY_OUT}")
    py_code = generate_py_module(COMMANDS)

    # Ensure directories exist
    JSON_OUT.parent.mkdir(parents=True, exist_ok=True)
    CPP_OUT.parent.mkdir(parents=True, exist_ok=True)
    PY_OUT.parent.mkdir(parents=True, exist_ok=True)

    JSON_OUT.write_text(json_text, encoding="utf-8")
    CPP_OUT.write_text(cpp_code, encoding="utf-8")
    PY_OUT.write_text(py_code, encoding="utf-8")

    print("[gen_commands] Done.")


if __name__ == "__main__":
    main()
