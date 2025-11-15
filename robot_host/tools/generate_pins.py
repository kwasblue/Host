#!/usr/bin/env python3
"""
Generate PinConfig.h (C++) and pin_config.py (Python)
from a single pins.json file.

Run:  python gen_pins.py
"""

import json
from pathlib import Path

# Adjust these three paths to match your layout
ROOT = Path(__file__).resolve().parent

PINS_JSON = ROOT / "pins.json"

# Where to write the C++ header (ESP32 firmware project)
CPP_OUT = Path("/Users/kwasiaddo/projects/PlatformIO/Projects/ESP32 MCU Host/include/config/PinConfig.h")


# Where to write the python header (ESP32 firmware project)
PY_OUT  = Path("/Users/kwasiaddo/projects/Host/robot_host/pin_config.py")


def load_pins(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("pins.json must be a JSON object {NAME: number, ...}")
    # Basic validation
    for name, value in data.items():
        if not isinstance(name, str):
            raise ValueError(f"Pin name {name!r} is not a string")
        if not isinstance(value, int):
            raise ValueError(f"Pin {name} value {value!r} is not an int")
        if not (0 <= value <= 39):
            raise ValueError(f"Pin {name} value {value} looks invalid for ESP32 GPIO")
    return data


def generate_cpp(pins: dict) -> str:
    lines = []
    lines.append("// AUTO-GENERATED FILE — DO NOT EDIT BY HAND")
    lines.append("// Generated from pins.json by gen_pins.py\n")
    lines.append("#pragma once")
    lines.append("#include <stdint.h>\n")
    lines.append("namespace Pins {")
    for name, value in sorted(pins.items()):
        lines.append(f"    constexpr uint8_t {name} = {value};")
    lines.append("} // namespace Pins")
    lines.append("")
    return "\n".join(lines)


def generate_py(pins: dict) -> str:
    lines = []
    lines.append("# AUTO-GENERATED FILE — DO NOT EDIT BY HAND")
    lines.append("# Generated from pins.json by gen_pins.py\n")
    for name, value in sorted(pins.items()):
        lines.append(f"{name} = {value}")
    lines.append("")
    return "\n".join(lines)


def main():
    print(f"[gen_pins] Loading {PINS_JSON}")
    pins = load_pins(PINS_JSON)

    cpp_code = generate_cpp(pins)
    py_code = generate_py(pins)

    CPP_OUT.parent.mkdir(parents=True, exist_ok=True)
    PY_OUT.parent.mkdir(parents=True, exist_ok=True)

    print(f"[gen_pins] Writing C++ header: {CPP_OUT}")
    CPP_OUT.write_text(cpp_code, encoding="utf-8")

    print(f"[gen_pins] Writing Python module: {PY_OUT}")
    PY_OUT.write_text(py_code, encoding="utf-8")

    print("[gen_pins] Done.")


if __name__ == "__main__":
    main()
