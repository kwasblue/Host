#!/usr/bin/env python3
"""
Generate command schema artifacts from platform_schema.COMMANDS.
"""

from pathlib import Path
import json
from platform_schema import ROOT, COMMANDS

JSON_OUT = ROOT / "commands.json"
CPP_OUT = Path("/Users/kwasiaddo/projects/PlatformIO/Projects/ESP32 MCU Host/include/config/CommandDefs.h")
PY_OUT  = ROOT / "Host/robot_host/command_defs.py"


def validate_commands(cmds: dict) -> None:
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

        for field_name, field_spec in payload.items():
            if not isinstance(field_spec, dict):
                raise ValueError(f"{name}.{field_name}: field spec must be an object")
            if "type" not in field_spec:
                raise ValueError(f"{name}.{field_name}: field must define 'type'")


def generate_json(commands: dict) -> str:
    schema = {
        "schema_version": 1,
        "commands": commands,
    }
    return json.dumps(schema, indent=2, sort_keys=True)


def generate_cpp_header(commands: dict) -> str:
    names = sorted(commands.keys())
    enum_entries = [name.replace("CMD_", "") for name in names]

    lines: list[str] = []
    lines.append("// AUTO-GENERATED FILE — DO NOT EDIT BY HAND")
    lines.append("// Generated from COMMANDS in platform_schema.py\n")
    lines.append("#pragma once")
    lines.append("#include <string>\n")
    lines.append("enum class CmdType {")
    for entry in enum_entries:
        lines.append(f"    {entry},")
    lines.append("    UNKNOWN")
    lines.append("};\n")

    lines.append("inline CmdType cmdTypeFromString(const std::string& s) {")
    for name, entry in zip(names, enum_entries):
        lines.append(f'    if (s == "{name}") return CmdType::{entry};')
    lines.append("    return CmdType::UNKNOWN;")
    lines.append("}\n")

    lines.append("inline const char* cmdTypeToString(CmdType c) {")
    lines.append("    switch (c) {")
    for name, entry in zip(names, enum_entries):
        lines.append(f'        case CmdType::{entry}: return "{name}";')
    lines.append('        default: return "UNKNOWN";')
    lines.append("    }")
    lines.append("}\n")

    return "\n".join(lines)


def generate_py_module(commands: dict) -> str:
    names = sorted(commands.keys())

    lines: list[str] = []
    lines.append("# AUTO-GENERATED FILE — DO NOT EDIT BY HAND")
    lines.append("# Generated from COMMANDS in platform_schema.py\n")

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

    JSON_OUT.parent.mkdir(parents=True, exist_ok=True)
    CPP_OUT.parent.mkdir(parents=True, exist_ok=True)
    PY_OUT.parent.mkdir(parents=True, exist_ok=True)

    JSON_OUT.write_text(json_text, encoding="utf-8")
    CPP_OUT.write_text(cpp_code, encoding="utf-8")
    PY_OUT.write_text(py_code, encoding="utf-8")

    print("[gen_commands] Done.")


if __name__ == "__main__":
    main()
