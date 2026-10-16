import math
from .constants import (
        BAUDRATE,
        HEAD_TX,
        HEAD_RX,
        RX_LEN,
        GEAR_RATIO,
        K_POS,
        K_SPEED,
        K_TORQUE,
        K_KP,
        K_KD,
        K_VOLTAGE,
        OUT_POS_RES,
        ERROR_MAP,
    )


def OutPosRaw2OutPos_rad(outpos: int) -> float:
    """将输出轴原始值转换为弧度"""
    return (outpos / OUT_POS_RES) * 2.0 * math.pi

def rad2deg(rad: float) -> float:
    """弧度转角度"""
    return rad * (180.0 / math.pi)
def deg2rad(deg: float) -> float:
    """角度转弧度"""
    return deg * (math.pi / 180.0)

def OutPos_rad2OutPosRaw(outpos_rad: float) -> int:
    """将输出轴弧度值转换为原始值"""
    return int((outpos_rad / (2.0 * math.pi)) * OUT_POS_RES)

