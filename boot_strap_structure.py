#!/usr/bin/env python3
"""
Bootstrap / normalize the robot_host package structure.

- Creates missing directories (core, modules, telemetry, runners, scratch, etc.).
- Ensures __init__.py files exist.
- Creates stub modules for:
    - core.settings
    - core.robot_runtime
    - modules.telemetry_ctl
    - modules.logging_ctl
    - modules.encoder
    - modules.motion
    - modules.modes
    - modules.gpio
    - modules.pwm
    - modules.servo
    - modules.stepper
- Adds a default robot_profile_default.yaml to config/ if missing.

Existing files are never overwritten.
"""

from __future__ import annotations

from pathlib import Path
import textwrap


ROOT = Path(__file__).resolve().parent
PKG = ROOT / "robot_host"


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def ensure_init(path: Path) -> None:
    init = path / "__init__.py"
    if not init.exists():
        init.write_text("# Auto-created package marker\n", encoding="utf-8")


def write_if_missing(path: Path, content: str) -> None:
    if path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(content).lstrip("\n"), encoding="utf-8")
    print(f"[bootstrap] created {path.relative_to(ROOT)}")


def main() -> None:
    if not PKG.exists():
        raise SystemExit(f"robot_host/ not found at {PKG}")

    # -------------------------------------------------------------------------
    # 1) Ensure directory skeleton + __init__
    # -------------------------------------------------------------------------
    dirs = [
        PKG,
        PKG / "core",
        PKG / "transports",
        PKG / "config",
        PKG / "modules",
        PKG / "telemetry",
        PKG / "runners",
        PKG / "scratch",
        PKG / "tools",  # already exists, but keep it explicit
    ]

    for d in dirs:
        ensure_dir(d)
        ensure_init(d)

    # -------------------------------------------------------------------------
    # 2) Core scaffolding: settings + robot_runtime
    # -------------------------------------------------------------------------

    settings_py = PKG / "core" / "settings.py"
    robot_runtime_py = PKG / "core" / "robot_runtime.py"

    write_if_missing(
        settings_py,
        """
        from dataclasses import dataclass
        from pathlib import Path
        from typing import Optional

        import yaml  # type: ignore[import]

        @dataclass
        class TransportSettings:
            type: str = "tcp"         # "tcp" | "serial" | "ble"
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

        @dataclass
        class EncoderDefaults:
            encoder_id: int = 0
            pin_a: int = 32
            pin_b: int = 33

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
        """
    )

    write_if_missing(
        robot_runtime_py,
        """
        from __future__ import annotations

        from dataclasses import dataclass

        from .event_bus import EventBus
        from .client import AsyncRobotClient
        from .settings import HostSettings

        from ..transports.tcp_transport import AsyncTcpTransport
        from ..transports.serial_transport import SerialTransport
        from ..transports.bluetooth_transport import BluetoothSerialTransport

        from ..telemetry.host_module import TelemetryHostModule
        from ..modules.encoder import EncoderHostModule, EncoderDefaults
        from ..modules.motion import MotionHostModule
        from ..modules.modes import ModeHostModule
        from ..modules.telemetry_ctl import TelemetryControlModule
        from ..modules.logging_ctl import LoggingControlModule


        @dataclass
        class RobotRuntime:
            bus: EventBus
            client: AsyncRobotClient
            telemetry: TelemetryHostModule | None = None
            encoder: EncoderHostModule | None = None
            motion: MotionHostModule | None = None
            modes: ModeHostModule | None = None
            telemetry_ctl: TelemetryControlModule | None = None
            logging_ctl: LoggingControlModule | None = None


        async def build_runtime(profile: str = "default") -> RobotRuntime:
            settings = HostSettings.load(profile)
            bus = EventBus()

            # --- Transport selection ---
            t_cfg = settings.transport
            if t_cfg.type == "tcp":
                transport = AsyncTcpTransport(host=t_cfg.host, port=t_cfg.port)
            elif t_cfg.type == "serial":
                transport = SerialTransport(
                    port=t_cfg.serial_port,
                    baudrate=t_cfg.baudrate,
                )
            elif t_cfg.type == "ble":
                transport = BluetoothSerialTransport.auto(
                    device_name=t_cfg.ble_name,
                    baudrate=t_cfg.baudrate,
                )
            else:
                raise ValueError(f"Unknown transport type: {t_cfg.type}")

            client = AsyncRobotClient(transport, bus=bus)
            await client.start()

            telemetry = TelemetryHostModule(bus) if settings.features.telemetry else None
            encoder = (
                EncoderHostModule(
                    bus,
                    client,
                    EncoderDefaults(
                        encoder_id=settings.encoder_defaults.encoder_id,
                        pin_a=settings.encoder_defaults.pin_a,
                        pin_b=settings.encoder_defaults.pin_b,
                    ),
                )
                if settings.features.encoder
                else None
            )

            motion = (
                MotionHostModule(bus, client)
                if settings.features.motion
                else None
            )
            modes = (
                ModeHostModule(bus, client)
                if settings.features.modes
                else None
            )

            telemetry_ctl = TelemetryControlModule(bus, client)
            logging_ctl = LoggingControlModule(bus, client)

            return RobotRuntime(
                bus=bus,
                client=client,
                telemetry=telemetry,
                encoder=encoder,
                motion=motion,
                modes=modes,
                telemetry_ctl=telemetry_ctl,
                logging_ctl=logging_ctl,
            )
        """
    )

    # -------------------------------------------------------------------------
    # 3) Modules scaffolding
    # -------------------------------------------------------------------------
    modules_dir = PKG / "modules"

    write_if_missing(
        modules_dir / "telemetry_ctl.py",
        """
        from typing import Any
        from robot_host.core.client import AsyncRobotClient
        from robot_host.core.event_bus import EventBus


        class TelemetryControlModule:
            \"\"\"Host-side helper for CMD_TELEM_SET_INTERVAL.\"\"\"

            def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
                self._bus = bus
                self._client = client

            async def set_interval(self, interval_ms: int) -> None:
                payload: dict[str, Any] = {"interval_ms": interval_ms}
                await self._client.send_json_cmd("CMD_TELEM_SET_INTERVAL", payload)
        """
    )

    write_if_missing(
        modules_dir / "logging_ctl.py",
        """
        from typing import Any
        from robot_host.core.client import AsyncRobotClient
        from robot_host.core.event_bus import EventBus


        class LoggingControlModule:
            \"\"\"Host-side helper for CMD_SET_LOG_LEVEL.\"\"\"

            def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
                self._bus = bus
                self._client = client

            async def set_level(self, level: str = "info") -> None:
                payload: dict[str, Any] = {"level": level}
                await self._client.send_json_cmd("CMD_SET_LOG_LEVEL", payload)
        """
    )

    write_if_missing(
        modules_dir / "encoder.py",
        """
        from __future__ import annotations

        from dataclasses import dataclass
        from typing import Any

        from robot_host.core.client import AsyncRobotClient
        from robot_host.core.event_bus import EventBus


        @dataclass
        class EncoderDefaults:
            encoder_id: int = 0
            pin_a: int = 32
            pin_b: int = 33


        class EncoderHostModule:
            \"\"\"Host-side wrapper around encoder commands.

            Expects CMD_ENCODER_ATTACH / CMD_ENCODER_READ / CMD_ENCODER_RESET
            to be implemented on the MCU.
            \"\"\"

            def __init__(
                self,
                bus: EventBus,
                client: AsyncRobotClient,
                defaults: EncoderDefaults | None = None,
            ) -> None:
                self._bus = bus
                self._client = client
                self._defaults = defaults or EncoderDefaults()

            async def attach(
                self,
                encoder_id: int | None = None,
                pin_a: int | None = None,
                pin_b: int | None = None,
            ) -> None:
                eid = encoder_id if encoder_id is not None else self._defaults.encoder_id
                payload: dict[str, Any] = {"encoder_id": eid}
                payload["pin_a"] = pin_a if pin_a is not None else self._defaults.pin_a
                payload["pin_b"] = pin_b if pin_b is not None else self._defaults.pin_b

                await self._client.send_json_cmd("CMD_ENCODER_ATTACH", payload)

            async def read(self, encoder_id: int | None = None) -> None:
                eid = encoder_id if encoder_id is not None else self._defaults.encoder_id
                payload: dict[str, Any] = {"encoder_id": eid}
                await self._client.send_json_cmd("CMD_ENCODER_READ", payload)

            async def reset(self, encoder_id: int | None = None) -> None:
                eid = encoder_id if encoder_id is not None else self._defaults.encoder_id
                payload: dict[str, Any] = {"encoder_id": eid}
                await self._client.send_json_cmd("CMD_ENCODER_RESET", payload)
        """
    )

    write_if_missing(
        modules_dir / "motion.py",
        """
        from typing import Any
        from robot_host.core.client import AsyncRobotClient
        from robot_host.core.event_bus import EventBus


        class MotionHostModule:
            \"\"\"High-level motion helpers (SET_VEL / STOP / ESTOP).\"\"\"

            def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
                self._bus = bus
                self._client = client

            async def set_velocity(self, vx: float, omega: float) -> None:
                payload: dict[str, Any] = {"vx": float(vx), "omega": float(omega)}
                await self._client.send_json_cmd("CMD_SET_VEL", payload)

            async def stop(self) -> None:
                await self._client.send_json_cmd("CMD_STOP", {})

            async def estop(self) -> None:
                await self._client.send_json_cmd("CMD_ESTOP", {})

            async def clear_estop(self) -> None:
                await self._client.send_json_cmd("CMD_CLEAR_ESTOP", {})
        """
    )

    write_if_missing(
        modules_dir / "modes.py",
        """
        from typing import Any
        from robot_host.core.client import AsyncRobotClient
        from robot_host.core.event_bus import EventBus


        class ModeHostModule:
            \"\"\"Wrapper for CMD_SET_MODE (IDLE / ARMED / ACTIVE / ESTOP).\"\"\"

            def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
                self._bus = bus
                self._client = client

            async def set_mode(self, mode: str) -> None:
                payload: dict[str, Any] = {"mode": mode}
                await self._client.send_json_cmd("CMD_SET_MODE", payload)
        """
    )

    # Skeletons for gpio/pwm/servo/stepper (you can fill later as needed)
    write_if_missing(
        modules_dir / "gpio.py",
        """
        from typing import Any
        from robot_host.core.client import AsyncRobotClient
        from robot_host.core.event_bus import EventBus


        class GpioHostModule:
            \"\"\"GPIO write/read/toggle helpers.\"\"\"

            def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
                self._bus = bus
                self._client = client

            async def write(self, channel: int, value: int) -> None:
                payload: dict[str, Any] = {"channel": int(channel), "value": int(value)}
                await self._client.send_json_cmd("CMD_GPIO_WRITE", payload)

            async def read(self, channel: int) -> None:
                payload: dict[str, Any] = {"channel": int(channel)}
                await self._client.send_json_cmd("CMD_GPIO_READ", payload)

            async def toggle(self, channel: int) -> None:
                payload: dict[str, Any] = {"channel": int(channel)}
                await self._client.send_json_cmd("CMD_GPIO_TOGGLE", payload)
        """
    )

    write_if_missing(
        modules_dir / "pwm.py",
        """
        from typing import Any
        from robot_host.core.client import AsyncRobotClient
        from robot_host.core.event_bus import EventBus


        class PwmHostModule:
            \"\"\"PWM_SET helper.\"\"\"

            def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
                self._bus = bus
                self._client = client

            async def set(self, channel: int, duty: float, freq_hz: float | None = None) -> None:
                payload: dict[str, Any] = {
                    "channel": int(channel),
                    "duty": float(duty),
                }
                if freq_hz is not None:
                    payload["freq_hz"] = float(freq_hz)
                await self._client.send_json_cmd("CMD_PWM_SET", payload)
        """
    )

    write_if_missing(
        modules_dir / "servo.py",
        """
        from typing import Any
        from robot_host.core.client import AsyncRobotClient
        from robot_host.core.event_bus import EventBus


        class ServoHostModule:
            \"\"\"Servo attach/set helpers.\"\"\"

            def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
                self._bus = bus
                self._client = client

            async def attach(self, servo_id: int = 0, min_us: int = 1000, max_us: int = 2000) -> None:
                payload: dict[str, Any] = {
                    "servo_id": int(servo_id),
                    "min_us": int(min_us),
                    "max_us": int(max_us),
                }
                await self._client.send_json_cmd("CMD_SERVO_ATTACH", payload)

            async def set_angle(self, servo_id: int, angle_deg: float, duration_ms: int = 0) -> None:
                payload: dict[str, Any] = {
                    "servo_id": int(servo_id),
                    "angle_deg": float(angle_deg),
                    "duration_ms": int(duration_ms),
                }
                await self._client.send_json_cmd("CMD_SERVO_SET_ANGLE", payload)
        """
    )

    write_if_missing(
        modules_dir / "stepper.py",
        """
        from typing import Any
        from robot_host.core.client import AsyncRobotClient
        from robot_host.core.event_bus import EventBus


        class StepperHostModule:
            \"\"\"Stepper move/enable helpers.\"\"\"

            def __init__(self, bus: EventBus, client: AsyncRobotClient) -> None:
                self._bus = bus
                self._client = client

            async def move_rel(self, motor_id: int, steps: int, speed_steps_s: float) -> None:
                payload: dict[str, Any] = {
                    "motor_id": int(motor_id),
                    "steps": int(steps),
                    "speed_steps_s": float(speed_steps_s),
                }
                await self._client.send_json_cmd("CMD_STEPPER_MOVE_REL", payload)

            async def enable(self, motor_id: int, enable: bool = True) -> None:
                payload: dict[str, Any] = {
                    "motor_id": int(motor_id),
                    "enable": bool(enable),
                }
                await self._client.send_json_cmd("CMD_STEPPER_ENABLE", payload)
        """
    )

    # -------------------------------------------------------------------------
    # 4) Telemetry package marker (if missing)
    # -------------------------------------------------------------------------
    telemetry_init = PKG / "telemetry" / "__init__.py"
    ensure_init(telemetry_init.parent)

    # -------------------------------------------------------------------------
    # 5) Default profile YAML (only if missing)
    # -------------------------------------------------------------------------
    default_profile = PKG / "config" / "robot_profile_default.yaml"
    if not default_profile.exists():
        default_profile.write_text(
            textwrap.dedent(
                """
                # Default robot host profile
                transport:
                  type: tcp
                  host: 10.0.0.61
                  port: 3333

                features:
                  telemetry: true
                  encoder: true
                  motion: false
                  modes: true
                  camera: false

                encoder_defaults:
                  encoder_id: 0
                  pin_a: 32
                  pin_b: 33
                """
            ).lstrip("\n"),
            encoding="utf-8",
        )
        print(f"[bootstrap] created config/{default_profile.name}")

    print("[bootstrap] done.")


if __name__ == "__main__":
    main()
