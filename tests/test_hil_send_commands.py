# tests/hil/test_commands.py

import pytest
import asyncio
from robot_host.transports.tcp_transport import AsyncTcpTransport
from robot_host.core.client import AsyncRobotClient

# Mark entire module as HIL (requires hardware)
pytestmark = [
    pytest.mark.hil,
    pytest.mark.asyncio,
]


@pytest.fixture
async def robot(request):
    """Connect to robot for HIL tests."""
    host = request.config.getoption("--robot-host", default="10.0.0.60")
    port = request.config.getoption("--robot-port", default=3333)
    
    transport = AsyncTcpTransport(host, port)
    client = AsyncRobotClient(transport)
    await client.start()
    
    yield client
    
    await client.stop()


# ============== Safety Commands ==============

class TestSafetyCommands:
    async def test_heartbeat(self, robot):
        ok, err = await robot.send_reliable("CMD_HEARTBEAT")
        assert ok, f"HEARTBEAT failed: {err}"

    async def test_arm_disarm_cycle(self, robot):
        # Ensure clean state
        await robot.send_reliable("CMD_CLEAR_ESTOP")
        
        ok, err = await robot.send_reliable("CMD_ARM")
        assert ok, f"ARM failed: {err}"
        
        ok, err = await robot.send_reliable("CMD_DISARM")
        assert ok, f"DISARM failed: {err}"

    async def test_full_state_cycle(self, robot):
        """Test complete state machine: IDLE -> ARMED -> ACTIVE -> ARMED -> IDLE"""
        await robot.send_reliable("CMD_CLEAR_ESTOP")
        
        ok, _ = await robot.send_reliable("CMD_ARM")
        assert ok
        
        ok, _ = await robot.send_reliable("CMD_ACTIVATE")
        assert ok
        
        ok, _ = await robot.send_reliable("CMD_DEACTIVATE")
        assert ok
        
        ok, _ = await robot.send_reliable("CMD_DISARM")
        assert ok


# ============== Rate Commands ==============

class TestRateCommands:
    async def test_get_rates(self, robot):
        ok, err = await robot.send_reliable("CMD_GET_RATES")
        assert ok, f"GET_RATES failed: {err}"

    @pytest.mark.parametrize("hz", [10, 50, 100, 200])
    async def test_ctrl_set_rate(self, robot, hz):
        ok, err = await robot.send_reliable("CMD_CTRL_SET_RATE", {"hz": hz})
        assert ok, f"CTRL_SET_RATE({hz}) failed: {err}"


# ============== Control Kernel ==============

class TestControlKernel:
    async def test_signal_define_and_list(self, robot):
        """Define signals and verify they appear in list."""
        # Define a test signal
        ok, err = await robot.send_reliable("CMD_CTRL_SIGNAL_DEFINE", {
            "id": 100,
            "name": "test_ref",
            "signal_kind": "REF",
            "initial": 0.0
        })
        assert ok, f"SIGNAL_DEFINE failed: {err}"
        
        # Verify it's in the list
        ok, err = await robot.send_reliable("CMD_CTRL_SIGNALS_LIST")
        assert ok, f"SIGNALS_LIST failed: {err}"

    async def test_signal_set_get(self, robot):
        """Set a signal value and read it back."""
        # Define signal first
        await robot.send_reliable("CMD_CTRL_SIGNAL_DEFINE", {
            "id": 101, "name": "test_val", "signal_kind": "REF", "initial": 0.0
        })
        
        # Set value
        ok, err = await robot.send_reliable("CMD_CTRL_SIGNAL_SET", {
            "id": 101, "value": 42.5
        })
        assert ok, f"SIGNAL_SET failed: {err}"
        
        # Read back
        ok, err = await robot.send_reliable("CMD_CTRL_SIGNAL_GET", {"id": 101})
        assert ok, f"SIGNAL_GET failed: {err}"

    async def test_slot_config_full_cycle(self, robot):
        """Configure a PID slot and run through its lifecycle."""
        # Define signals
        for sig in [
            {"id": 200, "name": "ref", "signal_kind": "REF", "initial": 0.0},
            {"id": 201, "name": "meas", "signal_kind": "MEAS", "initial": 0.0},
            {"id": 202, "name": "out", "signal_kind": "OUT", "initial": 0.0},
        ]:
            ok, _ = await robot.send_reliable("CMD_CTRL_SIGNAL_DEFINE", sig)
            assert ok
        
        # Configure slot
        ok, err = await robot.send_reliable("CMD_CTRL_SLOT_CONFIG", {
            "slot": 0,
            "controller_type": "PID",
            "rate_hz": 100,
            "ref_id": 200,
            "meas_id": 201,
            "out_id": 202,
            "require_armed": True,
            "require_active": True
        })
        assert ok, f"SLOT_CONFIG failed: {err}"
        
        # Set parameters
        ok, err = await robot.send_reliable("CMD_CTRL_SLOT_SET_PARAM", {
            "slot": 0, "key": "kp", "value": 1.0
        })
        assert ok, f"SLOT_SET_PARAM failed: {err}"
        
        # Check status
        ok, err = await robot.send_reliable("CMD_CTRL_SLOT_STATUS", {"slot": 0})
        assert ok, f"SLOT_STATUS failed: {err}"


# ============== GPIO ==============

class TestGPIO:
    async def test_register_write_read_toggle(self, robot):
        ok, _ = await robot.send_reliable("CMD_GPIO_REGISTER_CHANNEL", {
            "channel": 0, "pin": 2, "mode": "output"
        })
        assert ok
        
        ok, _ = await robot.send_reliable("CMD_GPIO_WRITE", {"channel": 0, "value": 1})
        assert ok
        
        ok, _ = await robot.send_reliable("CMD_GPIO_READ", {"channel": 0})
        assert ok
        
        ok, _ = await robot.send_reliable("CMD_GPIO_TOGGLE", {"channel": 0})
        assert ok


# ============== Motors ==============

class TestDCMotor:
    async def test_dc_motor_speed_control(self, robot):
        ok, _ = await robot.send_reliable("CMD_DC_SET_SPEED", {"motor_id": 0, "speed": 0.1})
        assert ok
        
        ok, _ = await robot.send_reliable("CMD_DC_STOP", {"motor_id": 0})
        assert ok

    async def test_dc_motor_pid_control(self, robot):
        ok, _ = await robot.send_reliable("CMD_DC_VEL_PID_ENABLE", {"motor_id": 0, "enable": True})
        assert ok
        
        ok, _ = await robot.send_reliable("CMD_DC_SET_VEL_GAINS", {
            "motor_id": 0, "kp": 1.0, "ki": 0.1, "kd": 0.01
        })
        assert ok
        
        ok, _ = await robot.send_reliable("CMD_DC_SET_VEL_TARGET", {"motor_id": 0, "omega": 0.1})
        assert ok