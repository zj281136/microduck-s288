#!/usr/bin/env python3
"""Read raw LSM6DSV16X accelerometer/gyro data over Raspberry Pi I2C.

This is a sensor-only bring-up tool. It never connects to the motor controller.
The reported axes are the sensor's native axes; robot-frame alignment is a
separate calibration step.
"""

from __future__ import annotations

import argparse
import math
import struct
import time

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

# ST lsm6dsv16x-pid conversion constants for ±4 g and ±1000 dps.
ACCEL_M_S2_PER_LSB = 0.122e-3 * 9.80665
GYRO_RAD_S_PER_LSB = 35.0e-3 * math.pi / 180.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x6A)
    parser.add_argument("--duration", type=float, default=0.0, help="0 runs until Ctrl+C")
    parser.add_argument("--print-rate", type=float, default=10.0)
    args = parser.parse_args()
    if args.address not in (0x6A, 0x6B):
        parser.error("--address must be 0x6a or 0x6b")
    if args.duration < 0 or args.print_rate <= 0:
        parser.error("duration must be >= 0 and print-rate must be > 0")
    return args


def update_bits(bus: SMBus, address: int, register: int, mask: int, value: int) -> None:
    current = bus.read_byte_data(address, register)
    bus.write_byte_data(address, register, (current & ~mask) | (value & mask))


def configure(bus: SMBus, address: int) -> None:
    device_id = bus.read_byte_data(address, WHO_AM_I)
    if device_id != EXPECTED_ID:
        raise RuntimeError(
            f"WHO_AM_I mismatch at 0x{address:02x}: "
            f"expected 0x{EXPECTED_ID:02x}, got 0x{device_id:02x}"
        )

    # BDU=1 and register auto-increment=1.
    update_bits(bus, address, CTRL3, 0x44, 0x44)
    # Gyroscope full scale ±1000 dps; accelerometer full scale ±4 g.
    update_bits(bus, address, CTRL6, 0x0F, 0x03)
    update_bits(bus, address, CTRL8, 0x03, 0x01)
    # 120 Hz ODR in high-performance mode for both sensors.
    update_bits(bus, address, CTRL1, 0x7F, 0x06)
    update_bits(bus, address, CTRL2, 0x7F, 0x06)
    time.sleep(0.05)


def power_down(bus: SMBus, address: int) -> None:
    update_bits(bus, address, CTRL1, 0x0F, 0x00)
    update_bits(bus, address, CTRL2, 0x0F, 0x00)


def read_sample(bus: SMBus, address: int):
    # One auto-incremented transaction keeps temperature, gyro and accel close
    # in time: temp[2], gyro xyz[6], accel xyz[6].
    block = bytes(bus.read_i2c_block_data(address, OUT_TEMP_L, 14))
    temp_raw, gx, gy, gz, ax, ay, az = struct.unpack("<7h", block)
    accel = tuple(value * ACCEL_M_S2_PER_LSB for value in (ax, ay, az))
    gyro = tuple(value * GYRO_RAD_S_PER_LSB for value in (gx, gy, gz))
    temperature = temp_raw / 256.0 + 25.0
    return accel, gyro, temperature


def main() -> int:
    args = parse_args()
    print(
        f"Opening /dev/i2c-{args.bus}, address=0x{args.address:02x}; "
        "ODR=120Hz, accel=±4g, gyro=±1000dps"
    )
    with SMBus(args.bus) as bus:
        configure(bus, args.address)
        print("WHO_AM_I=0x70 OK. Sensor axes are not yet mapped to robot axes.")
        started = time.monotonic()
        next_print = started + 1.0 / args.print_rate
        stat_started = started
        samples = 0
        stat_samples = 0
        try:
            while args.duration == 0 or time.monotonic() - started < args.duration:
                status = bus.read_byte_data(args.address, STATUS_REG)
                if status & 0x03 != 0x03:  # accelerometer and gyro both ready
                    time.sleep(0.0005)
                    continue
                accel, gyro, temperature = read_sample(bus, args.address)
                samples += 1
                stat_samples += 1
                now = time.monotonic()
                if now >= next_print:
                    accel_norm = math.sqrt(sum(value * value for value in accel))
                    elapsed = max(now - stat_started, 1e-9)
                    print(
                        f"a=[{accel[0]:+7.3f} {accel[1]:+7.3f} {accel[2]:+7.3f}]m/s² "
                        f"|a|={accel_norm:6.3f}  "
                        f"w=[{gyro[0]:+7.4f} {gyro[1]:+7.4f} {gyro[2]:+7.4f}]rad/s "
                        f"T={temperature:5.1f}C rate={stat_samples / elapsed:6.1f}Hz"
                    )
                    next_print = now + 1.0 / args.print_rate
                if now - stat_started >= 1.0:
                    stat_started = now
                    stat_samples = 0
        except KeyboardInterrupt:
            print("\nCtrl+C")
        finally:
            power_down(bus, args.address)
            print(f"Sensor powered down cleanly; samples={samples}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
