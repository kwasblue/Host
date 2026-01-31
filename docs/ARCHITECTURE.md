# MARA Host Architecture

**Modular Asynchronous Robotics Architecture - Python Host Component**

---

## Overview

The MARA Host library (`robot_host`) follows a layered, async-first architecture for controlling ESP32-based robots. This is the Python component of the MARA framework.

## Layer Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      User Application                            │
│                  (examples, custom code)                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                   AsyncRobotClient                       │    │
│  │        (main interface, lifecycle, state machine)        │    │
│  └───────────────────────────┬─────────────────────────────┘    │
│                              │                                   │
│  ┌───────────────────────────┴───────────────────────────┐      │
│  │                       EventBus                         │      │
│  │              (publish/subscribe messaging)             │      │
│  └─────────────────────────┬─────────────────────────────┘      │
│                            │                                     │
│  ┌──────────┬──────────┬───┴───┬──────────┬──────────┐          │
│  │ Command  │Telemetry │Module │ Research │Recording │          │
│  │  Layer   │  Layer   │ Layer │  Tools   │  Tools   │          │
│  ├──────────┼──────────┼───────┼──────────┼──────────┤          │
│  │ Reliable │ Parser   │Motion │Simulation│Recording │          │
│  │Commander │ Models   │GPIO   │ SysID    │ Replay   │          │
│  │Connection│HostMod  │Encoder│ Metrics  │ Analysis │          │
│  │ Monitor  │FileLog  │ IMU   │ Plotting │          │          │
│  └──────────┴──────────┴───────┴──────────┴──────────┘          │
│                            │                                     │
│  ┌─────────────────────────┴─────────────────────────────┐      │
│  │                    Transport Layer                     │      │
│  ├─────────────┬─────────────┬─────────────┬─────────────┤      │
│  │   Serial    │    TCP      │  Bluetooth  │   Stream    │      │
│  │  Transport  │  Transport  │  Transport  │  Transport  │      │
│  └─────────────┴─────────────┴─────────────┴─────────────┘      │
│                            │                                     │
│  ┌─────────────────────────┴─────────────────────────────┐      │
│  │                    Protocol Layer                      │      │
│  │               (frame encoding/decoding)                │      │
│  └───────────────────────────────────────────────────────┘      │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Core Components

### AsyncRobotClient

Main interface for robot communication:

```python
class AsyncRobotClient:
    def __init__(self, transport, bus=None, ...):
        self.transport = transport
        self.bus = bus or EventBus()
        self.connection = ConnectionMonitor(...)
        self.commander = ReliableCommander(...)

    async def start(self):
        # 1. Start transport
        # 2. Perform version handshake
        # 3. Start heartbeat loop
        # 4. Start connection monitor

    async def stop(self):
        # 1. Cancel heartbeat
        # 2. Stop monitors
        # 3. Close transport
```

### EventBus

Decoupled pub/sub messaging:

```python
class EventBus:
    def subscribe(self, topic: str, handler: Callable):
        self._handlers[topic].append(handler)

    def publish(self, topic: str, data: Any):
        for handler in self._handlers.get(topic, []):
            handler(data)
```

Common topics:
- `telemetry.*` - Sensor data
- `cmd.*` - Command ACKs
- `connection.*` - Connection events
- `heartbeat` - Keep-alive

### ReliableCommander

Guaranteed delivery with retries:

```python
class ReliableCommander:
    async def send(self, cmd_type, payload, wait_for_ack=True):
        seq = await self.send_func(cmd_type, payload)
        if wait_for_ack:
            future = self._pending[seq].future
            return await future  # (ok, error)
        return True, None

    def on_ack(self, seq, ok, error):
        cmd = self._pending.pop(seq)
        cmd.future.set_result((ok, error))
```

### Transport Layer

Abstract interface for communication:

```python
class HasSendBytes(Protocol):
    async def send_bytes(self, data: bytes) -> None: ...
    def set_frame_handler(self, handler: Callable[[bytes], None]) -> None: ...
    def start(self) -> object: ...
    def stop(self) -> object: ...
```

Implementations:
- `SerialTransport` - USB/UART serial
- `AsyncTcpTransport` - WiFi TCP with auto-reconnect
- `BluetoothTransport` - Bluetooth Classic

## Message Flow

### Command with ACK

```
AsyncRobotClient                ReliableCommander              Transport
      │                                │                            │
      │ send_reliable("CMD_ARM", {})   │                            │
      ├───────────────────────────────►│                            │
      │                                │   send_bytes(frame)        │
      │                                ├───────────────────────────►│
      │                                │                            │
      │                                │   (pending[seq] = Future)  │
      │                                │                            │
      │                                │◄────── ACK frame ──────────│
      │                                │                            │
      │                                │   on_ack(seq, ok, error)   │
      │                                │   future.set_result(...)   │
      │                                │                            │
      │◄───────────── (ok, error) ─────┤                            │
      │                                │                            │
```

### Telemetry Flow

```
Transport              EventBus              TelemetryHostModule
    │                      │                         │
    │ frame received       │                         │
    ├─────────────────────►│                         │
    │ publish("telemetry.raw", data)                 │
    │                      ├────────────────────────►│
    │                      │                         │ parse(data)
    │                      │                         │
    │                      │◄────────────────────────┤
    │                      │ publish("telemetry.imu", imu)
    │                      │                         │
    │                      │◄────────────────────────┤
    │                      │ publish("telemetry.encoder0", enc)
    │                      │                         │
```

## Protocol

### Frame Format

```
┌────────┬────────┬────────┬──────────┬─────────────┬──────────┐
│ HEADER │ LEN_HI │ LEN_LO │ MSG_TYPE │   PAYLOAD   │ CHECKSUM │
│  0xAA  │   1B   │   1B   │    1B    │   N bytes   │    1B    │
└────────┴────────┴────────┴──────────┴─────────────┴──────────┘
```

### Optimized Parsing

```python
_HEADER_BYTE = bytes([HEADER])

def extract_frames(buffer, on_frame):
    i = 0
    n = len(buffer)

    while i + MIN_FRAME_HEADER <= n:
        # Vectorized header search (faster than byte-by-byte)
        if buffer[i] != HEADER:
            idx = buffer.find(_HEADER_BYTE, i)
            if idx == -1:
                break
            i = idx

        # Parse frame...
```

## Research Module

### Simulation Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    SimulationRunner                          │
│              (coordinates simulation loop)                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────┐      ┌─────────────────┐               │
│  │  DiffDriveRobot │◄────►│   Controller    │               │
│  │   (physics)     │      │  (user-defined) │               │
│  └────────┬────────┘      └─────────────────┘               │
│           │                                                  │
│  ┌────────┴────────┐                                        │
│  │                 │                                        │
│  ▼                 ▼                                        │
│ ┌─────────┐  ┌──────────┐  ┌────────────┐                   │
│ │ DCMotor │  │ DCMotor  │  │ NoiseModels│                   │
│ │  Left   │  │  Right   │  │(IMU,Enc,US)│                   │
│ └─────────┘  └──────────┘  └────────────┘                   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Config Loading

```yaml
# robot_config.yaml
name: my_robot
type: diff_drive

drive:
  wheel_radius: 0.05
  wheel_base: 0.2

noise:
  imu:
    accel_std: 0.01
    gyro_std: 0.001
```

```python
robot = load_robot("robot_config.yaml")
```

## Optimization Techniques

### Lazy Dict Copying

```python
# Before: Copy on every update (150 objects/sec at 50Hz)
spec.latest_payload = dict(payload)

# After: Store reference, copy only when consumed
spec.latest_payload = payload
spec._payload_dirty = True

def _get_payload(spec):
    if spec._payload_dirty:
        result = dict(spec.latest_payload)
        spec._payload_dirty = False
        return result
    return spec.latest_payload
```

### Bounded Pending Commands

```python
MAX_PENDING_AGE_S = 30.0

async def _update(self):
    for seq, cmd in list(self._pending.items()):
        age = (now - cmd.first_sent_ns) / 1e9
        if age > MAX_PENDING_AGE_S:
            # Force evict stale commands (memory leak prevention)
            self._pending.pop(seq)
            cmd.future.set_result((False, "STALE"))
```

## Extending the Library

### Adding a New Module

```python
# robot_host/my_module/manager.py
from robot_host.command.client import AsyncRobotClient
from robot_host.core.event_bus import EventBus

class MyModuleHostModule:
    def __init__(self, bus: EventBus, client: AsyncRobotClient):
        self._bus = bus
        self._client = client
        bus.subscribe("telemetry.my_sensor", self._on_data)

    async def send_command(self, param: float):
        await self._client.cmd_my_command(param=param)

    def _on_data(self, data):
        # Process incoming data
        self._bus.publish("my_module.processed", processed_data)
```

### Adding a New Transport

```python
class MyTransport:
    def __init__(self, ...):
        self._frame_handler = lambda x: None

    def set_frame_handler(self, handler):
        self._frame_handler = handler

    async def start(self):
        # Initialize connection
        pass

    async def stop(self):
        # Close connection
        pass

    async def send_bytes(self, data: bytes):
        # Send data
        pass

    def _on_receive(self, data: bytes):
        # Called when data received
        extract_frames(self._buffer, self._frame_handler)
```

## Testing

```bash
# Run all tests
pytest tests/ -v

# Run with coverage
pytest tests/ --cov=robot_host

# Run specific test
pytest tests/test_protocol.py -v
```

Test structure:
```
tests/
├── test_protocol.py      # Frame encoding/decoding
├── test_event_bus.py     # Pub/sub system
├── test_client.py        # AsyncRobotClient
├── test_commander.py     # ReliableCommander
└── test_hil/             # Hardware-in-loop tests
```
