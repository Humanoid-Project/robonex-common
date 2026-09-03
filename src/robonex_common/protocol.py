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

COMM_DEVICE_ID = 0x00
COMM_OPERATION = 0x01
COMM_FEEDBACK = 0x02
COMM_ENABLE = 0x03
COMM_STOP = 0x04
COMM_SET_ZERO = 0x06
COMM_SET_CAN_ID = 0x07
COMM_PARAMETER_READ = 0x11
COMM_PARAMETER_WRITE = 0x12
COMM_SAVE = 0x16
COMM_ACTIVE_REPORT = 0x18

DEVICE_ID_DESTINATION = 0xFE
ZERO_STATUS_INDEX = 0x7029
VBUS_INDEX = 0x701C


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


FAULT_BIT_NAMES = {
    0: "Overtemperature (>145C)",
    1: "Driver chip fault",
    2: "Undervoltage (<12V)",
    3: "Overvoltage (>60V)",
    4: "Phase B overcurrent",
    5: "Phase C overcurrent",
    7: "Encoder not calibrated",
    8: "Hardware identification fault",
    9: "Position initialization fault",
    14: "Stall overload",
    16: "Phase A overcurrent",
}


def decode_fault_bits(value):
    return [name for bit, name in FAULT_BIT_NAMES.items() if value & (1 << bit)]
