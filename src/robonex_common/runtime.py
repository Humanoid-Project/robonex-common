try:
    import numpy as np
except ImportError:  # pragma: no cover
    raise ImportError("numpy is required; install robonex-common[policy]")

OBSERVATION_TERM_SIZES = (
    ("joint_pos_rel", 12),
    ("joint_vel_rel", 12),
    ("imu_ang_vel", 3),
    ("projected_gravity", 3),
    ("velocity_commands", 3),
    ("gait_phase", 2),
    ("last_action", 12),
)
OBSERVATION_FRAME_SIZE = sum(size for _, size in OBSERVATION_TERM_SIZES)
OBSERVATION_HISTORY_LENGTH = 5
OBSERVATION_SIZE = OBSERVATION_FRAME_SIZE * OBSERVATION_HISTORY_LENGTH
ACTION_SIZE = 12


def assemble_observation_frame(
    joint_pos, joint_vel, ang_vel, gravity, command, gait_phase, last_action, frame_size=None
):
    frame = np.concatenate(
        (
            np.asarray(joint_pos, dtype=np.float32).reshape(-1),
            np.asarray(joint_vel, dtype=np.float32).reshape(-1),
            np.asarray(ang_vel, dtype=np.float32).reshape(-1),
            np.asarray(gravity, dtype=np.float32).reshape(-1),
            np.asarray(command, dtype=np.float32).reshape(-1),
            np.asarray(gait_phase, dtype=np.float32).reshape(-1),
            np.asarray(last_action, dtype=np.float32).reshape(-1),
        )
    ).astype(np.float32)
    expected = OBSERVATION_FRAME_SIZE if frame_size is None else int(frame_size)
    if frame.shape != (expected,):
        raise ValueError(f"observation frame must have {expected} values, got {frame.shape[0]}")
    if not np.isfinite(frame).all():
        raise ValueError("observation frame contains a non-finite value")
    return frame


class ObservationHistory:
    """Isaac Lab observation history layout.

    Isaac Lab keeps one circular buffer per observation term and flattens each
    buffer on its own (oldest entry first) before concatenating the terms. The
    result is term-major, not frame-major, so a frame-major buffer would feed the
    policy a silently scrambled vector.
    """

    def __init__(self, term_sizes=OBSERVATION_TERM_SIZES, history_length=OBSERVATION_HISTORY_LENGTH):
        if history_length < 1:
            raise ValueError(f"history_length must be at least 1, got {history_length}")
        self.term_sizes = tuple((str(name), int(size)) for name, size in term_sizes)
        self.history_length = int(history_length)
        self.frame_size = sum(size for _, size in self.term_sizes)
        self.expected_size = self.frame_size * self.history_length
        self._frames = None

    @classmethod
    def from_contract(cls, contract):
        """Build the layout declared by a policy manifest.

        Each entry of ``observation_terms`` is ``name:size`` or ``name:sizexhistory``.
        """
        term_sizes = []
        history_length = None
        for entry in contract.observation_terms:
            name, _, spec = str(entry).partition(":")
            size, _, history = spec.partition("x")
            try:
                size = int(size)
                history = int(history) if history else 1
            except ValueError as error:
                raise ValueError(f"malformed observation term: {entry!r}") from error
            if history_length is None:
                history_length = history
            elif history_length != history:
                raise ValueError("every observation term must share one history length")
            term_sizes.append((name, size))
        layout = cls(term_sizes, history_length or 1)
        if layout.expected_size != int(contract.observation_size):
            raise ValueError(
                f"observation_terms sum to {layout.expected_size} but the manifest "
                f"declares observation_size={contract.observation_size}"
            )
        return layout

    def reset(self):
        self._frames = None

    def append(self, frame):
        frame = np.asarray(frame, dtype=np.float32).reshape(-1)
        if frame.shape != (self.frame_size,):
            raise ValueError(f"observation frame must have {self.frame_size} values, got {frame.shape[0]}")
        if self._frames is None:
            self._frames = np.repeat(frame[None, :], self.history_length, axis=0)
        else:
            self._frames = np.roll(self._frames, -1, axis=0)
            self._frames[-1] = frame
        return self

    def observation(self):
        if self._frames is None:
            raise RuntimeError("append() at least one frame before reading the observation")
        pieces = []
        offset = 0
        for _, size in self.term_sizes:
            pieces.append(self._frames[:, offset : offset + size].reshape(-1))
            offset += size
        observation = np.concatenate(pieces).astype(np.float32)
        if observation.shape != (self.expected_size,):
            raise ValueError(f"observation must have {self.expected_size} values, got {observation.shape[0]}")
        if not np.isfinite(observation).all():
            raise ValueError("observation contains a non-finite value")
        return observation


GAIT_PERIOD_S = 0.8


def gait_phase_at(step_index, period=GAIT_PERIOD_S, step_dt=1.0 / 50.0):
    """The (sin, cos) gait clock the policy was trained against.

    It is derived from the controller's own step counter, so it needs no sensor.
    Reset the counter whenever the observation history is reset.
    """
    phase = (float(step_index) * step_dt) % period / period
    angle = 2.0 * np.pi * phase
    return np.array([np.sin(angle), np.cos(angle)], dtype=np.float32)


def assemble_observation(
    joint_pos, joint_vel, ang_vel, gravity, command, gait_phase, last_action, history=None
):
    if history is None:
        history = ObservationHistory()
    frame = assemble_observation_frame(
        joint_pos, joint_vel, ang_vel, gravity, command, gait_phase, last_action,
        frame_size=history.frame_size,
    )
    return history.append(frame).observation()


class ActionPipeline:
    def __init__(self, contract):
        self.scales = np.asarray(contract.action_scales, dtype=np.float32)
        self.offsets = np.asarray(contract.action_offsets, dtype=np.float32)
        self.runner_clip = float(contract.runner_action_clip)
        self.target_low = np.asarray([pair[0] for pair in contract.target_clips], dtype=np.float32)
        self.target_high = np.asarray([pair[1] for pair in contract.target_clips], dtype=np.float32)
        self.action_size = int(contract.action_size)
        self.policy_call_count = 0
        self.runner_clip_count = 0
        self.target_clip_count = 0

    def apply(self, raw_action, max_raw_action=None):
        action = np.asarray(raw_action, dtype=np.float32).reshape(-1)
        if action.shape != (self.action_size,):
            raise ValueError(f"action must have {self.action_size} values, got {action.shape[0]}")
        if not np.isfinite(action).all():
            raise ValueError("action contains a non-finite value")
        if max_raw_action is not None:
            raw_max = float(np.max(np.abs(action)))
            if raw_max > max_raw_action:
                raise ValueError(f"Raw action limit exceeded: {raw_max:.6f} > {max_raw_action:.6f}")
        clipped = np.clip(action, -self.runner_clip, self.runner_clip)
        scaled = clipped * self.scales + self.offsets
        targets = np.clip(scaled, self.target_low, self.target_high)
        self.policy_call_count += 1
        self.runner_clip_count += int(np.count_nonzero(clipped != action))
        self.target_clip_count += int(np.count_nonzero(targets != scaled))
        return clipped, targets
