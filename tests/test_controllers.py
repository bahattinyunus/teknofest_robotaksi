"""
Unit tests – robotaksi_control PID + Stanley
"""
import math
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'robotaksi_control'))

from robotaksi_control.pid_controller import PID, StanleyController


# ──────────────────────────────────────────────────────────────────
# PID Tests
# ──────────────────────────────────────────────────────────────────

class TestPID:
    def test_proportional_only(self):
        pid = PID(kp=2.0, ki=0.0, kd=0.0, setpoint=10.0)
        output = pid.compute(8.0)
        # Error = 2, P = 4
        assert abs(output - 4.0) < 0.5

    def test_output_limits(self):
        pid = PID(kp=100.0, ki=0.0, kd=0.0, setpoint=100.0,
                  output_limits=(-1.0, 1.0))
        output = pid.compute(0.0)
        assert output == pytest.approx(1.0)

    def test_integrator_winds_down(self):
        pid = PID(kp=1.0, ki=0.5, kd=0.0, setpoint=0.0)
        # After many passes at zero error, integral should still be 0
        for _ in range(20):
            pid.compute(0.0)
        assert abs(pid._integral) < 0.01

    def test_no_derivative_spike_on_first_call(self):
        pid = PID(kp=0.0, ki=0.0, kd=10.0, setpoint=5.0,
                  output_limits=(-100.0, 100.0))
        # First call: no derivative contribution expected (prev_error = 0)
        out = pid.compute(5.0)
        assert abs(out) < 1.0


# ──────────────────────────────────────────────────────────────────
# Stanley Tests
# ──────────────────────────────────────────────────────────────────

class TestStanley:
    def test_zero_error(self):
        s = StanleyController(k=0.5)
        delta = s.compute(v=5.0, cte=0.0, yaw_error=0.0)
        assert delta == pytest.approx(0.0, abs=1e-6)

    def test_cte_turns_towards_lane(self):
        s = StanleyController(k=1.0)
        delta_positive = s.compute(v=1.0, cte=1.0, yaw_error=0.0)
        delta_negative = s.compute(v=1.0, cte=-1.0, yaw_error=0.0)
        assert delta_positive > 0
        assert delta_negative < 0

    def test_steering_clipped(self):
        s = StanleyController(k=10.0)
        delta = s.compute(v=0.1, cte=100.0, yaw_error=0.0)
        assert -0.5 <= delta <= 0.5

    def test_high_speed_reduces_correction(self):
        s = StanleyController(k=1.0)
        low_speed = s.compute(v=0.5, cte=1.0, yaw_error=0.0)
        high_speed = s.compute(v=10.0, cte=1.0, yaw_error=0.0)
        assert abs(low_speed) > abs(high_speed)
