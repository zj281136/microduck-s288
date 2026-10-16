"""Probe CLI coverage with a fake bus; no serial or motor access."""
import contextlib
import io
import json
from pathlib import Path
import runpy
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "teleop/scripts/probe_s288_bus.py"


class ProbeCliTest(unittest.TestCase):
    def run_probe(self, *args, missing=False):
        requests = []
        instances = []

        class FakeSDK:
            def __init__(self, motor_ids, port, **kwargs):
                self.motor_ids = motor_ids
                self.last_read_error = "timeout"
                self.closed = False
                instances.append(self)

            def write_control_packet(self, **kwargs):
                requests.append(kwargs)

            def read(self, expected_motor_id):
                if missing:
                    return None
                return SimpleNamespace(
                    raw_pos=0, raw_speed=0, raw_outpos=0, pos_rad=0.0,
                    speed_rads=0.0, OutPos_rad=0.0, torque_nm=0.0,
                    voltage=12.0, MError=0, ExFlag=0,
                )

            def close(self):
                self.closed = True

        fake = ModuleType("robot_control.arm.uniarm_l1_bus")
        fake.UnitreeMotorSDK = FakeSDK
        output = io.StringIO()
        with patch.dict(sys.modules, {fake.__name__: fake}), \
             patch.object(sys, "path", sys.path.copy()), \
             patch.object(sys, "argv", [str(SCRIPT), "--port", "FAKE", *args]), \
             contextlib.redirect_stdout(output), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as stopped:
                runpy.run_path(str(SCRIPT), run_name="__main__")
        rows = [json.loads(line) for line in output.getvalue().splitlines()]
        return stopped.exception.code, instances, requests, rows

    def test_default_covers_all_fifteen_with_zero_gains(self):
        code, buses, requests, rows = self.run_probe()
        self.assertEqual(code, 0)
        self.assertEqual(buses[0].motor_ids, list(range(15)))
        self.assertEqual([row["id"] for row in rows], list(range(15)) * 3)
        for request in requests:
            self.assertEqual([request[key] for key in ("kp", "kd", "torque", "speed")], [0] * 4)
        self.assertTrue(buses[0].closed)

    def test_can_probe_only_the_non_rl_servo(self):
        code, _, requests, rows = self.run_probe("--ids", "14", "--samples", "1")
        self.assertEqual(code, 0)
        self.assertEqual([row["id"] for row in rows], [14])
        self.assertEqual(requests[0]["motor_id"], 14)

    def test_out_of_range_ids_do_not_open_bus(self):
        for motor_id in ("-1", "15"):
            with self.subTest(motor_id=motor_id):
                code, buses, requests, _ = self.run_probe("--ids", motor_id)
                self.assertEqual(code, 2)
                self.assertEqual(buses, [])
                self.assertEqual(requests, [])

    def test_missing_feedback_is_reported_and_bus_closed(self):
        code, buses, _, rows = self.run_probe("--ids", "14", "--samples", "1", missing=True)
        self.assertEqual(code, 1)
        self.assertEqual(rows, [dict(id=14, tx_s=rows[0]["tx_s"], rx_s=rows[0]["rx_s"], error="timeout")])
        self.assertTrue(buses[0].closed)


if __name__ == "__main__":
    unittest.main()
