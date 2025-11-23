import asyncio

from robot_host.core.event_bus import EventBus
from robot_host.core.client import AsyncRobotClient
from robot_host.transports.tcp_transport import AsyncTcpTransport
from robot_host.transports.serial_transport import SerialTransport  # or serial_transport
from robot_host.telemetry.host_module import TelemetryHostModule
from robot_host.telemetry.models import ImuTelemetry, UltrasonicTelemetry, LidarTelemetry
from robot_host.transports.bluetooth_transport import BluetoothSerialTransport


def attach_telemetry_prints(bus: EventBus) -> None:
    # Pretty IMU print
    def on_imu(imu: ImuTelemetry) -> None:
        ax = imu.ax_g if imu.ax_g is not None else 0.0
        ay = imu.ay_g if imu.ay_g is not None else 0.0
        az = imu.az_g if imu.az_g is not None else 0.0
        temp = imu.temp_c if imu.temp_c is not None else 0.0

        print(
            f"[IMU] online={imu.online} ok={imu.ok} "
            f"ax={ax:+.3f}g ay={ay:+.3f}g az={az:+.3f}g "
            f"T={temp:.2f}°C"
        )


    def on_ultra(ultra: UltrasonicTelemetry) -> None:
        ts = f"{ultra.ts_ms}ms" if ultra.ts_ms is not None else "?"
        dist = f"{ultra.distance_cm:.1f} cm" if ultra.distance_cm is not None else "N/A"
        print(
            f"[ULTRA] t={ts} id={ultra.sensor_id} "
            f"attached={ultra.attached} ok={ultra.ok} d={dist}"
        )

    def on_lidar(lidar: LidarTelemetry) -> None:
        ts = f"{lidar.ts_ms}ms" if lidar.ts_ms is not None else "?"
        dist = f"{lidar.distance_m:.3f} m" if lidar.distance_m is not None else "N/A"
        sig = f"{lidar.signal:.1f}" if lidar.signal is not None else "N/A"

        print(
            f"[LIDAR] t={ts} online={lidar.online} ok={lidar.ok} "
            f"d={dist} signal={sig}"
        )
            

    bus.subscribe("telemetry.imu", on_imu)
    bus.subscribe("telemetry.ultrasonic", on_ultra)
    bus.subscribe("telemetry.lidar", on_lidar)


async def main() -> None:
    bus = EventBus()

    # Pick whatever transport you want here
    # For serial: from robot_host.transports.serial_transport import SerialTransport
    #transport = SerialTransport(port="/dev/cu.usbserial-0001", baudrate=115200)
    #transport = BluetoothSerialTransport.auto(device_name="ESP32-SPP",baudrate=11520)
    transport = AsyncTcpTransport(host="10.0.0.61", port=3333)  # adjust to your robot

    client = AsyncRobotClient(transport, bus=bus)
    

    # Structured telemetry module
    telemetry = TelemetryHostModule(bus)

    # Attach pretty printers
    attach_telemetry_prints(bus)

    await client.start()

    # Keep the process alive while the transport runs (if needed)
    # If your transport has its own loop this might not be necessary.
        # Toggle telemetry on/off every 30 seconds
    on = False
    INTERVAL_ON_MS = 100   # 10 Hz
    INTERVAL_OFF_MS = 0    # disabled

    while True:
        on = not on
        interval = INTERVAL_ON_MS if on else INTERVAL_OFF_MS
        state = "ON" if on else "OFF"
        print(f"\n[TELEM] Setting telemetry {state} (interval_ms={interval})")
        await client.cmd_telem_set_interval(interval_ms=interval)
        await asyncio.sleep(5.0)



if __name__ == "__main__":
    asyncio.run(main())
