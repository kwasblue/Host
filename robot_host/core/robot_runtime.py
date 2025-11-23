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
