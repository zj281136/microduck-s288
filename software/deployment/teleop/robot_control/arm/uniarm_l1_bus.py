# unitree.py 核心部分备份

import struct
import time
import serial
import math
import numpy as np
import logging
import threading
from dataclasses import dataclass
import sys
import time
from .constants import (
    BAUDRATE, HEAD_TX, HEAD_RX, RX_LEN, GEAR_RATIO,
    K_POS, K_SPEED, K_TORQUE, K_KP, K_KD, K_VOLTAGE,
    OUT_POS_RES, ERROR_MAP,CRC32_TABLE
)
from .utiles import (
    OutPosRaw2OutPos_rad, rad2deg, deg2rad, OutPos_rad2OutPosRaw,
)

logger = logging.getLogger(__name__)

@dataclass
class MotorState:
    id: int
    mode: int = 0
    timeout: int = 0
    temp_case: int = 0
    temp_winding: int = 0
    voltage: float = 0.0
    torque_nm: float = 0.0
    speed_rads: float = 0.0
    pos_rad: float = 0.0
    pos_raw_rad: int = 0
    # Unscaled fields from the last CRC-validated 26-byte feedback packet.
    # ``pos`` and ``speed`` are rotor-side; ``OutPos`` is output-side.
    raw_pos: int | None = None
    raw_speed: int | None = None
    raw_outpos: int | None = None
    MError: int = 0
    ExFlag: int = 0
    OutPos: int = 0
    OutPos_rad: float = 0.0
    OutPos_deg: float = 0.0
    CRC32: int = 0
    freq: float = 0.0
    last_update: float = time.perf_counter()
    pos_rad_threshold: float = 20.0


class UnitreeMotorSDK:
    """Unitree 电机 SDK - 单总线半双工通信，支持位置、速度、力矩控制模式，带 CRC 校验和连续位置跟踪"""

    def __init__(self, motor_ids, port: str, timeout: float = 0.01,
                 *, position_target_source: str = "outpos"):
        if position_target_source not in ("outpos", "rotor"):
            raise ValueError("position_target_source must be outpos or rotor")
        self.position_target_source = position_target_source
        self.ser = serial.Serial(
            port,
            BAUDRATE,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.motor_states = {motor_id: MotorState(id=motor_id) for motor_id in motor_ids}
        self.verbose_errors = False
        self._serial_lock = threading.RLock()
        self.last_valid_time = {}
        self.pos_rad_unwrapped = {motor_id: 0.0 for motor_id in motor_ids}  # 追踪 unwrapped 位置
        self.output_zero_offset = {motor_id: None for motor_id in motor_ids}
        self.position_command_reference = {}
        # read() keeps its historical MotorState-or-None return contract.  The
        # failure reason is exposed separately for diagnostics and statistics.
        self.last_read_error: str | None = None
        # Opt-in wire-command snapshots for non-actuating diagnostics.
        self.capture_commands = False
        self.last_control_commands = {}
    @staticmethod
    def _wrap_0_2pi(x):
        two_pi = 2.0 * math.pi
        return x % two_pi

    def _ensure_output_offset(self, motor_id: int):
        st = self.motor_states[motor_id]
        if self.output_zero_offset[motor_id] is None:
            raw_cont = st.pos_rad
            raw_mod = self._wrap_0_2pi(raw_cont)
            self.output_zero_offset[motor_id] = self._wrap_0_2pi(st.OutPos_rad - raw_mod)
    def get_output_pos_cont_aligned(self, motor_id: int) -> float:
        self._ensure_output_offset(motor_id)
        st = self.motor_states[motor_id]
        offset = self.output_zero_offset[motor_id]
        if offset is None:
            offset = 0.0
        return st.pos_rad + offset
    @staticmethod
    def crc32_lookup(data: bytes) -> int:
        crc = 0xFFFFFFFF
        n = len(data)
        i = 0
        while i + 3 < n:
            b0 = data[i + 3]
            b1 = data[i + 2]
            b2 = data[i + 1]
            b3 = data[i]
            crc = CRC32_TABLE[(crc >> 24) ^ b0] ^ ((crc << 8) & 0xFFFFFFFF)
            crc = CRC32_TABLE[(crc >> 24) ^ b1] ^ ((crc << 8) & 0xFFFFFFFF)
            crc = CRC32_TABLE[(crc >> 24) ^ b2] ^ ((crc << 8) & 0xFFFFFFFF)
            crc = CRC32_TABLE[(crc >> 24) ^ b3] ^ ((crc << 8) & 0xFFFFFFFF)
            i += 4
        return crc

    def write_control_packet(self, motor_id: int, pos: float = 0.0, kp: float = 1.8e-4,
            kd: float = 4.1e-5, torque: float = 0.0, speed: float = 0.5e-8,
            status: int = 1, timeout_enable: int = 0):
        """
        pos is output-equivalent radians in the selected encoder coordinate.
        Legacy outpos mode aligns OutPos to rotor on the first position command.
        Rotor mode uses only rotor feedback to select the nearest continuous turn.
        Both modes subsequently unwrap changes from the previous target.
        """

        mode_byte = (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout_enable & 0x01) << 7)
        st = self.motor_states[motor_id]

        two_pi = 2.0 * math.pi

        # 当前连续输出端位置（由返回的 pos 换算）
        cur_pos_cont = st.pos_rad

        # 当前输出端单圈角 [0, 2π)
        cur_pos_mod = st.OutPos_rad

        # Select the first reference in the SAME frame as the command.
        # Zero-gain polling must neither initialize nor move this reference.
        reference = None
        if kp != 0.0:
            if motor_id not in self.last_valid_time:
                raise RuntimeError(f"ID {motor_id}: valid feedback required before position control")
            previous = self.position_command_reference.get(motor_id)
            if previous is None:
                reference_pos = (cur_pos_cont if self.position_target_source == "rotor"
                                 else cur_pos_mod)
                previous = (reference_pos, cur_pos_cont)
            previous_output, previous_cont = previous
            delta = (pos - previous_output + math.pi) % two_pi - math.pi
            pos_cont = previous_cont + delta
            reference = (pos, pos_cont)
        else:
            pos_cont = cur_pos_cont

        raw_tor = max(-32768, min(32767, int((torque / GEAR_RATIO) * K_TORQUE)))
        raw_spd = max(-32768, min(32767, int((speed * GEAR_RATIO) * K_SPEED)))
        raw_pos = int((pos_cont * GEAR_RATIO) * K_POS)
        raw_kp = max(0, min(32767, int(kp * K_KP)))
        raw_kd = max(0, min(32767, int(kd * K_KD)))

        payload = struct.pack("<BBhhihh", mode_byte, 0, raw_tor, raw_spd, raw_pos, raw_kp, raw_kd)
        packet_without_crc = HEAD_TX + payload
        crc = self.crc32_lookup(packet_without_crc)
        full_packet = packet_without_crc + struct.pack('<I', crc)
        assert len(full_packet) == 20
        self.ser.write(full_packet)
        if reference is not None:
            self.position_command_reference[motor_id] = reference
        if self.capture_commands:
            self.last_control_commands[motor_id] = {
                "command_tx_s": time.perf_counter(),
                "pre_motor_cont_rad": cur_pos_cont,
                "pre_output_rad": cur_pos_mod,
                "requested_output_rad": pos,
                "requested_position_rad": pos,
                "position_target_source": self.position_target_source,
                "target_cont_rad": pos_cont,
                "wire_target_raw": raw_pos,
                "wire_target_cont_rad": raw_pos / K_POS / GEAR_RATIO,
                "sdk_kp": kp, "sdk_kd": kd,
                "wire_kp_raw": raw_kp, "wire_kd_raw": raw_kd,
            }
        

    def read(self, expected_motor_id: int | None = None):
        """Read one feedback packet, returning ``None`` on a bounded failure.

        ``last_read_error`` is one of timeout, short_frame, bad_header,
        bad_crc, or id_mismatch when the return value is ``None``.
        """
        self.last_read_error = None
        data = self.ser.read(RX_LEN)
        if not data:
            self.last_read_error = "timeout"
            return None
        if len(data) != RX_LEN:
            self.last_read_error = "short_frame"
            return None
        if data[:2] != HEAD_RX:
            self.last_read_error = "bad_header"
            return None

        mode_byte = data[2]
        motor_id = mode_byte & 0x0F
        mode = (mode_byte >> 4) & 0x07
        timeout = (mode_byte >> 7) & 0x01
        # 提取 19 字节的 fbk 数据
        fbk = data[3:22]  # 22 - 3 = 19
        if len(fbk) != 19:
            return None

        try:
            temp, sensor, vol, torque, speed, pos, MError, outpos_exflag, exsensor2, excom = \
                struct.unpack('<bBBhh i I H BB', fbk)
        except struct.error as e:
            print(f"[解析错误] {e}, fbk length={len(fbk)}")
            self.last_read_error = "short_frame"
            return None

        # 分离位域
        OutPos = outpos_exflag & 0x1FFF      # 低13位
        ExFlag = (outpos_exflag >> 13) & 0x07  # 高3位

        # CRC 校验：从 mode_byte 开始共 20 字节（data[2:22]）
        crc_received = struct.unpack('<I', data[22:26])[0]
        crc_computed = self.crc32_lookup(data[2:22])
        if crc_received != crc_computed:
            self.last_read_error = "bad_crc"
            return None
        if expected_motor_id is not None and motor_id != expected_motor_id:
            self.last_read_error = "id_mismatch"
            return None
        if motor_id not in self.motor_states:
            self.last_read_error = "id_mismatch"
            return None

        # 转换为物理量
        vol_f = vol / 2.0
        ExPos = 2 * math.pi * OutPos / 8192.0
        outputSpd = (speed / 2.56) * 2 * math.pi / GEAR_RATIO
        outputTor = (torque / 256000.0) * GEAR_RATIO
        outputPos = 2 * math.pi * pos / 32768.0 / GEAR_RATIO
        

        now = time.perf_counter()
        st = self.motor_states[motor_id]
        st.mode = mode
        st.timeout = timeout
        st.temp_case = temp   # 电机外壳温度
        st.temp_winding = sensor  # 电机绕组温度
        st.voltage = vol_f   # 电压
        st.torque_nm = outputTor
        st.speed_rads = outputSpd
        st.pos_rad = outputPos
        st.raw_pos = pos
        st.raw_speed = speed
        st.raw_outpos = OutPos
        st.MError = MError
        st.ExFlag = ExFlag
        st.OutPos = OutPos
        st.OutPos_rad = ExPos 
        st.OutPos_deg = rad2deg(ExPos)
        st.CRC32 = crc_received
        st.last_update = now
        
        # 频率计算
        last_time = self.last_valid_time.get(motor_id, now - 0.01)
        dt = now - last_time
        st.freq = (1.0 / dt) if dt > 0 else 0.0
        self.last_valid_time[motor_id] = now

        return st


    def parse_feedback_packet(self,data: bytes):
        if len(data) != 26:
            return None
        if data[0] != 0xFC or data[1] != 0xEE:
            return None

        mode_byte = data[2]
        motor_id = mode_byte & 0x0F
        mode = (mode_byte >> 4) & 0x07
        timeout = (mode_byte >> 7) & 0x01

        # 提取 19 字节的 fbk 数据
        fbk = data[3:22]  # 22 - 3 = 19
        if len(fbk) != 19:
            return None

        try:
            temp, sensor, vol, torque, speed, pos, MError, outpos_exflag, exsensor2, excom = \
                struct.unpack('<bBBhh i I H BB', fbk)
        except struct.error as e:
            print(f"[解析错误] {e}, fbk length={len(fbk)}")
            return None

        # 分离位域
        OutPos = outpos_exflag & 0x1FFF      # 低13位
        ExFlag = (outpos_exflag >> 13) & 0x07  # 高3位

        # CRC 校验：从 mode_byte 开始共 20 字节（data[2:22]）
        crc_received = struct.unpack('<I', data[22:26])[0]
        crc_computed = self.crc32_lookup(data[2:22])
        if crc_received != crc_computed:
            return None

        # 转换为物理量
        vol_f = vol / 2.0
        ExPos = 2 * math.pi * OutPos / 8192.0
        outputSpd = (speed / 2.56) * 2 * math.pi / GEAR_RATIO
        outputTor = (torque / 256000.0) * GEAR_RATIO
        outputPos = 2 * math.pi * pos / 32768.0 / GEAR_RATIO

        return {
            'motor_id': motor_id,
            'mode': mode,
            'timeout': timeout,
            'Temp': temp,
            'sensor': sensor,
            'vol': vol_f,
            'MError': MError,
            'MWarn': ExFlag,
            'ExPos': ExPos,
            'outputTor': outputTor,
            'outputSpd': outputSpd,
            'outputPos': outputPos
        }

    def send_and_receive(self, motor_id, mode, timeout_flag,
                         outputTor, outputSpd, outputPos, Kp, Kd):
        # 1. 发送控制包
        self.write_control_packet(
            motor_id=motor_id,
            pos=outputPos,
            kp=Kp,
            kd=Kd,
            torque=outputTor,
            speed=outputSpd,
            status=mode,
            timeout_enable=timeout_flag,
        )

        # 2. 立即读取并校验对应电机反馈（阻塞时间受串口 timeout 限制）
        return self.read(expected_motor_id=motor_id)
    def close(self):
        if self.ser.is_open:
            self.ser.close()


class UnitreeMotorsBus:
    """Unitree 半双工单总线"""

    def __init__(self, port: str, motor_ids: list[int],
                 kp_default: list | None = None,
                 kd_default: list | None = None,
                 torque_default: list | None = None,
                 speed_default: list | None = None,
                 kp_loop: list | None = None,
                 kd_loop: list | None = None,
                 handshake_timeout: float = 1.0):
        self.port = port
        self.motor_ids = motor_ids
        n_motors = len(motor_ids)

        # 每个电机独立参数（8个电机，按电机ID索引）
        # 默认值：M0-M7
        if kp_default is None:
            kp_default = [5.4755e-5, 5.6755e-5, 5.0755e-5, 6.0755e-5, 3.4755e-5, 2.6755e-5, 5.6755e-5, 8.0755e-5]
        if kd_default is None:
            kd_default = [9.6889e-6, 1.889e-5, 9.0889e-6, 6.6889e-6, 9.6889e-6, 0.9889e-5, 0.6889e-5, 8.0889e-6]
        if torque_default is None:
            torque_default = [0.0] * n_motors
        if speed_default is None:
            speed_default = [0.0] * n_motors
        if kp_loop is None:
            kp_loop = [1.0, 1.2, 0.9, 1.0, 1.0, 1.0, 0.9, 0.8]
        if kd_loop is None:
            kd_loop = [0.01, 0.04, 0.01, 0.01, 0.01, 0.01, 0.03, 0.01]

        self.kp_default = kp_default
        self.kd_default = kd_default
        self.torque_default = torque_default
        self.speed_default = speed_default
        self.kp_loop = kp_loop
        self.kd_loop = kd_loop

        self.handshake_timeout = handshake_timeout

        self.motors = UnitreeMotorSDK(motor_ids=motor_ids, port=port)
        self._is_connected = False

        self.con_mod = np.zeros(n_motors)
        self.tgt_pos_rad = np.zeros(n_motors)
        self.tgt_torque = np.zeros(n_motors)
        self.cur_pos_rad = np.zeros(n_motors)

        self.control_mode = 'zero_damping'  # 默认控制模式
        self.kp_torque_pos = 3.21
        self.kd_torque_vel = 0.17
        self.max_torque = 2.0
        self.range_min_rad = None
        self.range_max_rad = None

        # 连续 None 计数器（用于检测电机未连接）
        self._consecutive_none_count = 0
        self._max_consecutive_none = 20
        self._connection_error = None  # 存储连接错误信息

    def close_control(self):
        """发送控制指令"""
        debug_counter = 0

        tgt_pos_rad = np.zeros_like(self.tgt_pos_rad) if self.control_mode == 'zero_damping' else self.tgt_pos_rad
        kp= [0.0]*len(self.kp_default) if self.control_mode == 'zero_damping' else self.kp_default
        kd = [0.0]*len(self.kd_default) if self.control_mode == 'zero_damping' else self.kd_default
        torque = [0.0]*len(self.torque_default) if self.control_mode == 'zero_damping' else self.tgt_torque
        speed = [0.0]*len(self.speed_default) if self.control_mode == 'zero_damping' else self.speed_default
        if self.control_mode == 'zero_damping':
            status = np.ones(len(self.motor_ids))
        else:
            status = self.con_mod.copy()

        # # 极限位置安全裕度（弧度），在到达极限前提前停止
        # SAFETY_MARGIN = 0.05  # 约 3 度

        debug_counter += 1
        for i, mid in enumerate(self.motor_ids):
            cur_pos = self.motors.motor_states[mid].OutPos_rad
            freq = self.motors.motor_states[mid].freq


            target = tgt_pos_rad[i]

            delta = target - cur_pos
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi

            adjusted_target = cur_pos + self.kp_loop[i]*delta + self.kd_loop[i]*(0.0 - self.motors.motor_states[mid].speed_rads)

            
            self.motors.write_control_packet(
                    motor_id=mid,
                    pos=adjusted_target,
                    kp=kp[mid],
                    kd=kd[mid],
                    torque=torque[mid],
                    speed=speed[mid],
                    status=int(status[i]),
                    timeout_enable=0,
                )
            
            result = self.motors.read()

            # 检测连续 None（电机未连接）
            if result is None:
                self._consecutive_none_count += 1
                if self._consecutive_none_count >= self._max_consecutive_none:
                    print(f"   请检查电机电源和串口连接 ({self.port})")
                    import os
                    os._exit(1)
            else:
                self._consecutive_none_count = 0  # 重置计数器


    def set_all_zero(self):
        """一问一答模式：发一条 -> 收一条"""
        
        out = {}
        
        # 对每个电机：发送 -> 立刻读取
        for mid in self.motor_ids:
            self.motors.write_control_packet(
                motor_id=mid,
                status=1,
                kp=0.0,
                kd=0.0,
                pos=0.0,
                timeout_enable=0
            )
            
            # 立刻读取这条电机的回包
            fb = self.motors.read()
            
        # 收集所有电机的最新状态
        for mid in self.motor_ids:
            st = self.motors.motor_states.get(mid)
            if st is not None:
                out[mid] = {
                    "pos_raw_rad": st.pos_rad,
                    "out_pos_deg": st.OutPos_deg,
                    "OutPos_raw": st.OutPos,
                }
        
        return out

 



    def set_zero_damping(self):
        """设置所有电机为0阻尼模式（采集数据专用）"""
        results = []
        
        for mid in self.motor_ids:
            self.motors.write_control_packet(
                motor_id=mid,
                status=1,
                kp=0.0,
                kd=0.0,
                pos=0.0,
                timeout_enable=0,
            )
            self.motors.read()
            
            st = self.motors.motor_states.get(mid)
            if st:
                results.append({
                    "motor_id": mid,
                    "OutPos": st.OutPos,
                    "pos_rad": st.OutPos_rad,
                    "pos_deg": st.OutPos_deg,
                    "freq": st.freq,
                })
            else:
                results.append({
                    "motor_id": mid,
                    "OutPos": None,
                    "pos_rad": None,
                    "pos_deg": None,
                    "freq": 0.0,
                })
            
        
        return results
