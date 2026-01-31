#!/usr/bin/env python3
"""
Reorganize robot_host project into a cleaner package structure.

Expected starting layout (relevant bits):

Host/
  robot_host/
    generate_pins.py
    stream_transport.py
    tcp_transport.py
    serial_transport.py
    transport.py
    gen_commands.py
    bluetooth_transport.py
    protocol.py
    client.py
    pin_config.py
    run_serial_client.py
    event_bus.py
    run_tcp_client.py
    messages.py
    base_transport.py
    Host/robot_host/command_defs.py   # weird nested one

This script will create:

robot_host/
  core/
    event_bus.py
    messages.py
    protocol.py
    client.py
    command_defs.py
  transports/
    base_transport.py
    serial_transport.py
    tcp_transport.py
    bluetooth_transport.py
    stream_transport.py
    transport.py
  config/
    pin_config.py
  tools/
    generate_pins.py
    gen_commands.py
  runners/
    run_serial_client.py
    run_tcp_client.py
  __init__.py, core/__init__.py, ...

And remove __pycache__ dirs under robot_host.
"""

from pathlib import Path
import shutil

def ensure_dir(path: Path):
    if not path.exists():
        print(f"[mkdir] {path}")
        path.mkdir(parents=True, exist_ok=True)

def move_if_exists(src: Path, dst: Path):
    if not src.exists():
        print(f"[skip ] {src} (does not exist)")
        return
    ensure_dir(dst.parent)
    print(f"[move ] {src} -> {dst}")
    shutil.move(str(src), str(dst))

def touch_init(path: Path):
    if not path.exists():
        print(f"[touch] {path}")
        path.write_text("", encoding="utf-8")

def remove_pycache(root: Path):
    for p in root.rglob("__pycache__"):
        if p.is_dir():
            print(f"[rm   ] {p}")
            shutil.rmtree(p)

def main():
    # Assume this script lives in Host/ and robot_host is a sibling directory.
    script_dir = Path(__file__).resolve().parent
    robot_root = script_dir / "robot_host"

    if not robot_root.exists():
        raise SystemExit(f"robot_host directory not found at {robot_root}")

    print(f"[info ] Using robot_host directory: {robot_root}")

    core_dir       = robot_root / "core"
    transports_dir = robot_root / "transports"
    config_dir     = robot_root / "config"
    tools_dir      = robot_root / "tools"
    runners_dir    = robot_root / "runners"

    # Create directories
    for d in (core_dir, transports_dir, config_dir, tools_dir, runners_dir):
        ensure_dir(d)

    # --- Move core files ---
    move_if_exists(robot_root / "event_bus.py",    core_dir / "event_bus.py")
    move_if_exists(robot_root / "messages.py",     core_dir / "messages.py")
    move_if_exists(robot_root / "protocol.py",     core_dir / "protocol.py")
    move_if_exists(robot_root / "client.py",       core_dir / "client.py")

    # weird nested command_defs.py: robot_host/Host/robot_host/command_defs.py
    nested_cmd_defs = robot_root / "Host" / "robot_host" / "command_defs.py"
    if nested_cmd_defs.exists():
        move_if_exists(nested_cmd_defs, core_dir / "command_defs.py")
        # Optionally clean up empty Host/robot_host dirs
        host_dir = robot_root / "Host" / "robot_host"
        try:
            if host_dir.exists() and not any(host_dir.iterdir()):
                print(f"[rmdir] {host_dir}")
                host_dir.rmdir()
            parent = robot_root / "Host"
            if parent.exists() and not any(parent.iterdir()):
                print(f"[rmdir] {parent}")
                parent.rmdir()
        except OSError:
            # not empty or cannot remove; ignore
            pass
    else:
        print(f"[skip ] {nested_cmd_defs} (no nested command_defs.py found)")

    # --- Move transports ---
    move_if_exists(robot_root / "base_transport.py",   transports_dir / "base_transport.py")
    move_if_exists(robot_root / "serial_transport.py", transports_dir / "serial_transport.py")
    move_if_exists(robot_root / "tcp_transport.py",    transports_dir / "tcp_transport.py")
    move_if_exists(robot_root / "bluetooth_transport.py", transports_dir / "bluetooth_transport.py")
    move_if_exists(robot_root / "stream_transport.py", transports_dir / "stream_transport.py")
    move_if_exists(robot_root / "transport.py",        transports_dir / "transport.py")

    # --- Move config & tools ---
    move_if_exists(robot_root / "pin_config.py",    config_dir / "pin_config.py")
    move_if_exists(robot_root / "generate_pins.py", tools_dir / "generate_pins.py")
    move_if_exists(robot_root / "gen_commands.py",  tools_dir / "gen_commands.py")

    # --- Move runners ---
    move_if_exists(robot_root / "run_serial_client.py", runners_dir / "run_serial_client.py")
    move_if_exists(robot_root / "run_tcp_client.py",    runners_dir / "run_tcp_client.py")

    # --- Create __init__.py files ---
    touch_init(robot_root / "__init__.py")
    touch_init(core_dir / "__init__.py")
    touch_init(transports_dir / "__init__.py")
    touch_init(config_dir / "__init__.py")
    touch_init(tools_dir / "__init__.py")
    touch_init(runners_dir / "__init__.py")

    # --- Remove __pycache__ directories under robot_host ---
    remove_pycache(robot_root)

    print("[done ] Reorganization complete.")
    print("        NOTE: you still need to update imports to use package-style paths, e.g.")
    print("        from robot_host.command.client import RobotClient")

if __name__ == "__main__":
    main()
