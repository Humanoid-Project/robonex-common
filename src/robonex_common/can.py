import struct
import time

from .motors import MOTOR_SPECS
from .protocol import (
    COMM_ACTIVE_REPORT,
    COMM_ENABLE,
    COMM_FEEDBACK,
    COMM_OPERATION,
    COMM_PARAMETER_READ,
    COMM_PARAMETER_WRITE,
    COMM_STOP,
    HOST_ID,
    MECHANICAL_POSITION_INDEX,
    RUN_MODE_INDEX,
    RUN_MODE_OPERATION,
    build_arbitration_id,
    float_to_uint,
    parse_arbitration_id,
    uint_to_float,
)

try:
    import can as python_can
except ImportError:
    python_can = None


def drain(bus):
    dropped = 0
    while bus.recv(timeout=0.0) is not None:
        dropped += 1
    return dropped


class Motor:
    def __init__(self, bus, motor_id, spec, host_id=HOST_ID):
        self.bus = bus
        self.motor_id = motor_id
        self.spec = MOTOR_SPECS[spec] if isinstance(spec, str) else spec
        self.host_id = host_id
        self.last_position = None
        self.last_velocity = 0.0
        self.last_torque = 0.0
        self.last_temp = 0.0
        self.last_fault = 0
        self.last_mode_status = 0
        self.last_feedback_time = 0.0

    def _send(self, comm_type, data16, data):
        if python_can is None:
            raise RuntimeError("python-can is required; install robonex-common[can]")
        self.bus.send(
            python_can.Message(
                arbitration_id=build_arbitration_id(comm_type, data16, self.motor_id),
                data=bytes(data),
                is_extended_id=True,
            )
        )

    def enable(self):
        self._send(COMM_ENABLE, self.host_id, bytes(8))

    def stop(self, clear_fault=False):
        data = bytearray(8)
        data[0] = 1 if clear_fault else 0
        self._send(COMM_STOP, self.host_id, data)

    def control(self, pos, vel, kp, kd, torque=0.0):
        spec = self.spec
        data16 = float_to_uint(torque, spec.t_min, spec.t_max, 16)
        raw_pos = float_to_uint(pos, spec.p_min, spec.p_max, 16)
        raw_vel = float_to_uint(vel, spec.v_min, spec.v_max, 16)
        raw_kp = float_to_uint(kp, 0.0, spec.kp_max, 16)
        raw_kd = float_to_uint(kd, 0.0, spec.kd_max, 16)
        data = bytes(
            [
                (raw_pos >> 8) & 0xFF,
                raw_pos & 0xFF,
                (raw_vel >> 8) & 0xFF,
                raw_vel & 0xFF,
                (raw_kp >> 8) & 0xFF,
                raw_kp & 0xFF,
                (raw_kd >> 8) & 0xFF,
                raw_kd & 0xFF,
            ]
        )
        self._send(COMM_OPERATION, data16, data)

    def write_parameter_u8(self, index, value):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, index)
        data[4] = value & 0xFF
        self._send(COMM_PARAMETER_WRITE, self.host_id, data)

    def write_parameter_u16(self, index, value):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, index)
        struct.pack_into("<H", data, 4, value)
        self._send(COMM_PARAMETER_WRITE, self.host_id, data)

    def write_parameter_f32(self, index, value):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, index)
        struct.pack_into("<f", data, 4, value)
        self._send(COMM_PARAMETER_WRITE, self.host_id, data)

    def write_param_u8(self, index, value):
        self.write_parameter_u8(index, value)

    def write_param_u16(self, index, value):
        self.write_parameter_u16(index, value)

    def write_param_f32(self, index, value):
        self.write_parameter_f32(index, value)

    def set_active_report(self, enable):
        data = bytearray(8)
        data[0] = 1 if enable else 0
        self._send(COMM_ACTIVE_REPORT, self.host_id, data)

    def read_parameter(self, index, fmt="<f", timeout=0.2):
        data = bytearray(8)
        struct.pack_into("<H", data, 0, index)
        self._send(COMM_PARAMETER_READ, self.host_id, data)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None or not msg.is_extended_id:
                continue
            comm_type, data16, destination = parse_arbitration_id(msg.arbitration_id)
            if comm_type != COMM_PARAMETER_READ or destination != self.host_id or (data16 & 0xFF) != self.motor_id:
                continue
            payload = bytes(msg.data)
            if len(payload) >= 8 and int.from_bytes(payload[0:2], "little") == index:
                return struct.unpack_from(fmt, payload, 4)[0]
        return None

    def read_param(self, index, fmt="<f", timeout=0.2):
        return self.read_parameter(index, fmt=fmt, timeout=timeout)

    def read_param_f32(self, index, timeout=0.2):
        return self.read_parameter(index, fmt="<f", timeout=timeout)

    def read_mechanical_position(self, timeout=0.2):
        return self.read_parameter(MECHANICAL_POSITION_INDEX, timeout=timeout)

    def read_mech_position(self, timeout=0.2):
        return self.read_mechanical_position(timeout=timeout)

    def write_run_mode_operation(self):
        self.write_parameter_u8(RUN_MODE_INDEX, RUN_MODE_OPERATION)

    def ingest_feedback(self, data, data16=0, now=None):
        if len(data) < 8:
            return None
        spec = self.spec
        raw_pos = (data[0] << 8) | data[1]
        raw_vel = (data[2] << 8) | data[3]
        raw_torque = (data[4] << 8) | data[5]
        raw_temp = (data[6] << 8) | data[7]
        self.last_position = uint_to_float(raw_pos, spec.p_min, spec.p_max, 16)
        self.last_velocity = uint_to_float(raw_vel, spec.v_min, spec.v_max, 16)
        self.last_torque = uint_to_float(raw_torque, spec.t_min, spec.t_max, 16)
        self.last_temp = raw_temp / 10.0
        self.last_fault = (data16 >> 8) & 0x3F
        self.last_mode_status = (data16 >> 14) & 0x03
        self.last_feedback_time = time.monotonic() if now is None else now
        return (
            self.last_feedback_time,
            self.last_position,
            self.last_velocity,
            self.last_torque,
            self.last_temp,
            self.last_fault,
        )

    def poll_feedback(self, timeout=0.05):
        deadline = time.monotonic() + timeout
        first = True
        while first or time.monotonic() < deadline:
            first = False
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                if timeout <= 0.0:
                    return None
                continue
            if not msg.is_extended_id:
                continue
            comm_type, data16, destination = parse_arbitration_id(msg.arbitration_id)
            if comm_type != COMM_FEEDBACK or destination != self.host_id or (data16 & 0xFF) != self.motor_id:
                continue
            return self.ingest_feedback(bytes(msg.data), data16=data16)
        return None


class FeedbackHub:
    def __init__(self, bus, motors, host_id=HOST_ID):
        self.bus = bus
        self.motors = {motor.motor_id: motor for motor in motors}
        self.host_id = host_id

    def route(self, msg, now=None):
        if not msg.is_extended_id:
            return None
        comm_type, data16, destination = parse_arbitration_id(msg.arbitration_id)
        if comm_type != COMM_FEEDBACK or destination != self.host_id:
            return None
        motor = self.motors.get(data16 & 0xFF)
        if motor is not None:
            motor.ingest_feedback(bytes(msg.data), data16=data16, now=time.monotonic() if now is None else now)
        return motor

    def pump(self, max_frames=512):
        now = time.monotonic()
        for _ in range(max_frames):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                return
            self.route(msg, now)

    def wait_for(self, motor_id, timeout):
        deadline = time.monotonic() + timeout
        target = self.motors.get(motor_id)
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                return None
            if self.route(msg) is target and target is not None:
                return target.last_position
        return None
