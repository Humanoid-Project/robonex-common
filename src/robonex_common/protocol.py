HOST_ID = 0xFD
DEFAULT_INTERFACE = "socketcan"

RUN_MODE_INDEX = 0x7005
SPEED_REFERENCE_INDEX = 0x700A
TORQUE_LIMIT_INDEX = 0x700B
CURRENT_LIMIT_INDEX = 0x7018
MECHANICAL_POSITION_INDEX = 0x7019
MECHANICAL_VELOCITY_INDEX = 0x701B
ACCELERATION_INDEX = 0x7022
EPSCAN_TIME_INDEX = 0x7026
FAULT_STATUS_INDEX = 0x3022

RUN_MODE_OPERATION = 0
RUN_MODE_VELOCITY = 2

COMM_OPERATION = 0x01
COMM_FEEDBACK = 0x02
COMM_ENABLE = 0x03
COMM_STOP = 0x04
COMM_PARAMETER_READ = 0x11
COMM_PARAMETER_WRITE = 0x12
COMM_ACTIVE_REPORT = 0x18


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def float_to_uint(value, lower, upper, bits):
    value = clamp(value, lower, upper)
    return int((value - lower) / (upper - lower) * ((1 << bits) - 1))


def uint_to_float(raw, lower, upper, bits):
    return raw / float((1 << bits) - 1) * (upper - lower) + lower


def build_arbitration_id(comm_type, data16, target_id):
    return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)


def parse_arbitration_id(arbitration_id):
    return (
        (arbitration_id >> 24) & 0x1F,
        (arbitration_id >> 8) & 0xFFFF,
        arbitration_id & 0xFF,
    )


build_arb = build_arbitration_id
parse_arb = parse_arbitration_id
