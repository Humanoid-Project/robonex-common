try:
    import numpy as np
except ImportError:  # pragma: no cover
    raise ImportError("numpy is required; install robonex-common[policy]")

OBSERVATION_SIZE = 42
ACTION_SIZE = 12


def assemble_observation(joint_pos, joint_vel, ang_vel, gravity, last_action, expected_size=OBSERVATION_SIZE):
    observation = np.concatenate(
        (
            np.asarray(joint_pos, dtype=np.float32).reshape(-1),
            np.asarray(joint_vel, dtype=np.float32).reshape(-1),
            np.asarray(ang_vel, dtype=np.float32).reshape(-1),
            np.asarray(gravity, dtype=np.float32).reshape(-1),
            np.asarray(last_action, dtype=np.float32).reshape(-1),
        )
    ).astype(np.float32)
    if observation.shape != (expected_size,):
        raise ValueError(f"observation must have {expected_size} values, got {observation.shape[0]}")
    if not np.isfinite(observation).all():
        raise ValueError("observation contains a non-finite value")
    return observation


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
