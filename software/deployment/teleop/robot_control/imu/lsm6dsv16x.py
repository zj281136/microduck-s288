"""Minimal LSM6DSV16X I2C driver and body-frame gravity estimator.

The first hardware version assumes that sensor X/Y/Z are mounted parallel to
the robot's forward/left/up axes.  The accelerometer corrects gyro drift with a
low bandwidth complementary filter; it is not used as an instantaneous tilt
measurement while the robot is moving.
"""

from __future__ import annotations

import math
import struct
import threading
import time
import warnings
from dataclasses import dataclass

import numpy as np
from smbus2 import SMBus


WHO_AM_I = 0x0F
EXPECTED_ID = 0x70
CTRL1 = 0x10
CTRL2 = 0x11
CTRL3 = 0x12
CTRL6 = 0x15
CTRL8 = 0x17
STATUS_REG = 0x1E
OUT_TEMP_L = 0x20
ACCEL_M_S2_PER_LSB = 0.122e-3 * 9.80665  # +/-4 g
GYRO_RAD_S_PER_LSB = 35.0e-3 * math.pi / 180.0  # +/-1000 dps
MAX_CONSECUTIVE_IO_ERRORS = 3
IO_RETRY_DELAY_S = 0.002


class ImuError(RuntimeError):
    """The IMU could not provide a trustworthy observation."""


@dataclass(frozen=True)
class ImuSample:
    timestamp: float
    accel_m_s2: np.ndarray
    angular_velocity_rad_s: np.ndarray
    projected_gravity: np.ndarray
    temperature_c: float


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ImuError("cannot normalize IMU vector")
    return vector / norm


class LSM6DSV16X:
    """Continuously sample one LSM6DSV16X from a background thread."""

    def __init__(
        self,
        bus: int = 1,
        address: int = 0x6A,
        *,
        correction_tau_s: float = 1.5,
    ) -> None:
        if address not in (0x6A, 0x6B):
            raise ValueError("LSM6DSV16X address must be 0x6a or 0x6b")
        if correction_tau_s <= 0:
            raise ValueError("correction_tau_s must be positive")
        self.bus_number = bus
        self.address = address
        self.correction_tau_s = correction_tau_s
        self._bus: SMBus | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest: ImuSample | None = None
        self._thread_error: BaseException | None = None
        self._gyro_bias = np.zeros(3, dtype=np.float64)
        self._gravity = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)

    @staticmethod
    def _update_bits(bus: SMBus, address: int, register: int, mask: int, value: int) -> None:
        current = bus.read_byte_data(address, register)
        bus.write_byte_data(address, register, (current & ~mask) | (value & mask))

    def _configure(self) -> None:
        assert self._bus is not None
        device_id = self._bus.read_byte_data(self.address, WHO_AM_I)
        if device_id != EXPECTED_ID:
            raise ImuError(
                f"WHO_AM_I mismatch: expected 0x{EXPECTED_ID:02x}, got 0x{device_id:02x}"
            )
        self._update_bits(self._bus, self.address, CTRL3, 0x44, 0x44)  # BDU + auto increment
        self._update_bits(self._bus, self.address, CTRL6, 0x0F, 0x03)  # gyro +/-1000 dps
        self._update_bits(self._bus, self.address, CTRL8, 0x03, 0x01)  # accel +/-4 g
        self._update_bits(self._bus, self.address, CTRL1, 0x7F, 0x06)  # accel 120 Hz
        self._update_bits(self._bus, self.address, CTRL2, 0x7F, 0x06)  # gyro 120 Hz
        time.sleep(0.05)

    def _power_down(self) -> None:
        if self._bus is None:
            return
        self._update_bits(self._bus, self.address, CTRL1, 0x0F, 0x00)
        self._update_bits(self._bus, self.address, CTRL2, 0x0F, 0x00)

    def _read_raw(self) -> tuple[np.ndarray, np.ndarray, float]:
        assert self._bus is not None
        block = bytes(self._bus.read_i2c_block_data(self.address, OUT_TEMP_L, 14))
        temp_raw, gx, gy, gz, ax, ay, az = struct.unpack("<7h", block)
        accel = np.asarray((ax, ay, az), dtype=np.float64) * ACCEL_M_S2_PER_LSB
        gyro = np.asarray((gx, gy, gz), dtype=np.float64) * GYRO_RAD_S_PER_LSB
        return accel, gyro, temp_raw / 256.0 + 25.0

    def _wait_sample(self, deadline: float) -> tuple[np.ndarray, np.ndarray, float]:
        assert self._bus is not None
        while time.monotonic() < deadline:
            if self._bus.read_byte_data(self.address, STATUS_REG) & 0x03 == 0x03:
                return self._read_raw()
            time.sleep(0.0005)
        raise ImuError("timed out waiting for accelerometer and gyro")

    def start(self, calibration_seconds: float = 3.0) -> None:
        if self._bus is not None:
            raise ImuError("IMU is already open")
        if calibration_seconds < 1.0:
            raise ValueError("gyro calibration must be at least 1 second")
        self._bus = SMBus(self.bus_number)
        try:
            self._configure()
            gyros: list[np.ndarray] = []
            accels: list[np.ndarray] = []
            deadline = time.monotonic() + calibration_seconds
            while time.monotonic() < deadline:
                accel, gyro, _ = self._wait_sample(time.monotonic() + 0.1)
                gyros.append(gyro)
                accels.append(accel)
            gyro_array = np.asarray(gyros)
            accel_array = np.asarray(accels)
            if len(gyro_array) < 50:
                raise ImuError(f"only {len(gyro_array)} samples during calibration")
            gyro_rms = float(np.sqrt(np.mean(np.sum(gyro_array * gyro_array, axis=1))))
            accel_mean = np.mean(accel_array, axis=0)
            accel_norm = float(np.linalg.norm(accel_mean))
            if gyro_rms > 0.12:
                raise ImuError(f"robot moved during gyro calibration (RMS={gyro_rms:.3f} rad/s)")
            if not 8.5 <= accel_norm <= 11.0:
                raise ImuError(f"invalid stationary acceleration magnitude {accel_norm:.3f} m/s^2")
            self._gyro_bias = np.mean(gyro_array, axis=0)
            self._gravity = -_unit(accel_mean)
            self._stop.clear()
            self._thread_error = None
            self._thread = threading.Thread(target=self._sample_loop, name="lsm6dsv16x", daemon=True)
            self._thread.start()
        except BaseException:
            self.close()
            raise

    def _sample_loop(self) -> None:
        previous = time.monotonic()
        consecutive_io_errors = 0
        try:
            while not self._stop.is_set():
                assert self._bus is not None
                try:
                    if self._bus.read_byte_data(self.address, STATUS_REG) & 0x03 != 0x03:
                        time.sleep(0.0005)
                        continue
                    accel, gyro_raw, temperature = self._read_raw()
                    consecutive_io_errors = 0
                except OSError as exc:
                    consecutive_io_errors += 1
                    if consecutive_io_errors >= MAX_CONSECUTIVE_IO_ERRORS:
                        raise ImuError(
                            "I2C communication failed "
                            f"{consecutive_io_errors} consecutive times: {exc}"
                        ) from exc
                    time.sleep(IO_RETRY_DELAY_S)
                    continue
                now = time.monotonic()
                dt = min(0.05, max(1.0e-4, now - previous))
                previous = now
                gyro = gyro_raw - self._gyro_bias

                # A world-fixed vector represented in the rotating body frame:
                # dg_body/dt = -omega_body x g_body.
                predicted = _unit(self._gravity - np.cross(gyro, self._gravity) * dt)
                accel_norm = float(np.linalg.norm(accel))
                if 7.5 <= accel_norm <= 12.0:
                    accel_gravity = -accel / accel_norm
                    blend = min(0.02, dt / self.correction_tau_s)
                    self._gravity = _unit((1.0 - blend) * predicted + blend * accel_gravity)
                else:
                    self._gravity = predicted
                sample = ImuSample(
                    timestamp=now,
                    accel_m_s2=accel.astype(np.float32),
                    angular_velocity_rad_s=gyro.astype(np.float32),
                    projected_gravity=self._gravity.astype(np.float32),
                    temperature_c=float(temperature),
                )
                with self._lock:
                    self._latest = sample
        except BaseException as exc:
            self._thread_error = exc
            self._stop.set()

    def latest(self, max_age_s: float = 0.05) -> ImuSample:
        if self._thread_error is not None:
            raise ImuError(f"IMU sampling stopped: {self._thread_error}")
        with self._lock:
            sample = self._latest
        if sample is None:
            raise ImuError("no IMU sample available")
        age = time.monotonic() - sample.timestamp
        if age > max_age_s:
            raise ImuError(f"stale IMU sample ({age * 1000.0:.1f} ms)")
        return sample

    @property
    def gyro_bias(self) -> np.ndarray:
        return self._gyro_bias.astype(np.float32).copy()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self._thread = None
        try:
            try:
                self._power_down()
            except OSError as exc:
                # A disconnected/noisy IMU commonly rejects the power-down
                # write.  Shutdown must still close the bus, and this cleanup
                # error must not replace the actual runtime failure.
                warnings.warn(f"IMU power-down write failed: {exc}", RuntimeWarning)
        finally:
            if self._bus is not None:
                self._bus.close()
            self._bus = None

    def __enter__(self) -> "LSM6DSV16X":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
