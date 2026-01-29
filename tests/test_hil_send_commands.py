# tests/test_hil_commands.py

import pytest

pytestmark = [pytest.mark.hil, pytest.mark.asyncio]


class TestSafety:
    async def test_heartbeat(self, hil):
        await hil.assert_ok("CMD_HEARTBEAT")

    async def test_clear_estop(self, hil):
        await hil.assert_ok("CMD_CLEAR_ESTOP")

    async def test_arm_disarm(self, hil):
        await hil.assert_ok("CMD_ARM")
        await hil.assert_ok("CMD_DISARM")

    async def test_full_state_cycle(self, hil):
        await hil.assert_ok("CMD_ARM")
        await hil.assert_ok("CMD_ACTIVATE")
        await hil.assert_ok("CMD_DEACTIVATE")
        await hil.assert_ok("CMD_DISARM")


class TestRates:
    async def test_get_rates(self, hil):
        await hil.assert_ok("CMD_GET_RATES")

    @pytest.mark.parametrize("hz", [10, 50, 100])
    async def test_ctrl_set_rate(self, hil, hz):
        await hil.assert_ok("CMD_CTRL_SET_RATE", {"hz": hz})

    @pytest.mark.parametrize("hz", [10, 50, 100])
    async def test_safety_set_rate(self, hil, hz):
        await hil.assert_ok("CMD_SAFETY_SET_RATE", {"hz": hz})

    @pytest.mark.parametrize("hz", [1, 10, 50])
    async def test_telem_set_rate(self, hil, hz):
        await hil.assert_ok("CMD_TELEM_SET_RATE", {"hz": hz})


class TestControlSignals:
    async def test_define_signal(self, hil):
        await hil.assert_ok("CMD_CTRL_SIGNAL_DEFINE", {
            "id": 100, "name": "test_ref", "signal_kind": "REF", "initial": 0.0
        })

    async def test_signal_set_get(self, hil):
        await hil.assert_ok("CMD_CTRL_SIGNAL_DEFINE", {
            "id": 110, "name": "sg_test", "signal_kind": "REF", "initial": 0.0
        })
        await hil.assert_ok("CMD_CTRL_SIGNAL_SET", {"id": 110, "value": 42.5})
        await hil.assert_ok("CMD_CTRL_SIGNAL_GET", {"id": 110})

    async def test_signals_list(self, hil):
        await hil.assert_ok("CMD_CTRL_SIGNALS_LIST")


class TestControlSlots:
    @pytest.fixture
    async def slot(self, hil):
        """Configure slot 0 with signals."""
        for sig in [
            {"id": 200, "name": "s_ref", "signal_kind": "REF", "initial": 0.0},
            {"id": 201, "name": "s_meas", "signal_kind": "MEAS", "initial": 0.0},
            {"id": 202, "name": "s_out", "signal_kind": "OUT", "initial": 0.0},
        ]:
            await hil.assert_ok("CMD_CTRL_SIGNAL_DEFINE", sig)
        
        await hil.assert_ok("CMD_CTRL_SLOT_CONFIG", {
            "slot": 0, "controller_type": "PID", "rate_hz": 100,
            "ref_id": 200, "meas_id": 201, "out_id": 202,
        })
        return 0

    async def test_slot_enable(self, hil, slot):
        await hil.assert_ok("CMD_CTRL_SLOT_ENABLE", {"slot": slot, "enable": True})

    async def test_slot_reset(self, hil, slot):
        await hil.assert_ok("CMD_CTRL_SLOT_RESET", {"slot": slot})

    async def test_slot_set_param(self, hil, slot):
        await hil.assert_ok("CMD_CTRL_SLOT_SET_PARAM", {"slot": slot, "key": "kp", "value": 1.0})

    async def test_slot_status(self, hil, slot):
        await hil.assert_ok("CMD_CTRL_SLOT_STATUS", {"slot": slot})


class TestGPIO:
    async def test_register_write_read_toggle(self, hil):
        await hil.assert_ok("CMD_GPIO_REGISTER_CHANNEL", {"channel": 0, "pin": 2, "mode": "output"})
        await hil.assert_ok("CMD_GPIO_WRITE", {"channel": 0, "value": 1})
        await hil.assert_ok("CMD_GPIO_READ", {"channel": 0})
        await hil.assert_ok("CMD_GPIO_TOGGLE", {"channel": 0})


class TestPWM:
    async def test_pwm_set(self, hil):
        await hil.assert_ok("CMD_PWM_SET", {"channel": 0, "duty": 0.5, "freq_hz": 1000.0})


class TestLED:
    async def test_led_on_off(self, hil):
        await hil.assert_ok("CMD_LED_ON")
        await hil.assert_ok("CMD_LED_OFF")


class TestServo:
    async def test_attach_detach(self, hil):
        await hil.assert_ok("CMD_SERVO_ATTACH", {"servo_id": 0, "channel": 0, "min_us": 1000, "max_us": 2000})
        await hil.assert_ok("CMD_SERVO_DETACH", {"servo_id": 0})

    @pytest.mark.motion
    async def test_set_angle(self, active_robot, hil):
        await hil.assert_ok("CMD_SERVO_ATTACH", {"servo_id": 0, "channel": 0, "min_us": 1000, "max_us": 2000})
        await hil.assert_ok("CMD_SERVO_SET_ANGLE", {"servo_id": 0, "angle_deg": 90})


class TestStepper:
    async def test_enable_stop(self, hil):
        await hil.assert_ok("CMD_STEPPER_ENABLE", {"motor_id": 0, "enable": True})
        await hil.assert_ok("CMD_STEPPER_STOP", {"motor_id": 0})

    @pytest.mark.motion
    async def test_move_rel(self, active_robot, hil):
        await hil.assert_ok("CMD_STEPPER_ENABLE", {"motor_id": 0, "enable": True})
        await hil.assert_ok("CMD_STEPPER_MOVE_REL", {"motor_id": 0, "steps": 50, "speed_steps_s": 1000.0})


class TestEncoder:
    async def test_attach_read_reset(self, hil):
        await hil.assert_ok("CMD_ENCODER_ATTACH", {"encoder_id": 0, "pin_a": 32, "pin_b": 33})
        await hil.assert_ok("CMD_ENCODER_READ", {"encoder_id": 0})
        await hil.assert_ok("CMD_ENCODER_RESET", {"encoder_id": 0})


class TestDCMotor:
    async def test_stop(self, hil):
        await hil.assert_ok("CMD_DC_STOP", {"motor_id": 0})

    async def test_pid_gains(self, hil):
        await hil.assert_ok("CMD_DC_SET_VEL_GAINS", {"motor_id": 0, "kp": 1.0, "ki": 0.1, "kd": 0.01})

    @pytest.mark.motion
    async def test_set_speed(self, active_robot, hil):
        await hil.assert_ok("CMD_DC_SET_SPEED", {"motor_id": 0, "speed": 0.1})
        await hil.assert_ok("CMD_DC_STOP", {"motor_id": 0})


class TestMotion:
    @pytest.mark.motion
    async def test_set_vel(self, active_robot, hil):
        await hil.assert_ok("CMD_SET_VEL", {"vx": 0.0, "omega": 0.0})


class TestTelemetry:
    async def test_set_interval(self, hil):
        await hil.assert_ok("CMD_TELEM_SET_INTERVAL", {"interval_ms": 100})


class TestLogging:
    async def test_set_log_level(self, hil):
        await hil.assert_ok("CMD_SET_LOG_LEVEL", {"level": "info"})