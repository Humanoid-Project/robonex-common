import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

from .joints import JOINT_BY_MODEL_NAME, PASSIVE_CLOSED_LOOP_JOINTS


@dataclass(frozen=True)
class PolicyContract:
    schema_version: int
    policy_file: str
    policy_sha256: str
    joint_order: tuple[str, ...]
    action_offsets: tuple[float, ...]
    action_scales: tuple[float, ...]
    target_clips: tuple[tuple[float, float], ...]
    runner_action_clip: float
    observation_size: int
    action_size: int
    policy_hz: float
    description_commit: str
    common_commit: str

    @classmethod
    def from_dict(cls, data):
        contract = cls(
            schema_version=int(data["schema_version"]),
            policy_file=str(data["policy_file"]),
            policy_sha256=str(data["policy_sha256"]),
            joint_order=tuple(data["joint_order"]),
            action_offsets=tuple(float(value) for value in data["action_offsets"]),
            action_scales=tuple(float(value) for value in data["action_scales"]),
            target_clips=tuple((float(bounds[0]), float(bounds[1])) for bounds in data["target_clips"]),
            runner_action_clip=float(data["runner_action_clip"]),
            observation_size=int(data["observation_size"]),
            action_size=int(data["action_size"]),
            policy_hz=float(data["policy_hz"]),
            description_commit=str(data["description_commit"]),
            common_commit=str(data["common_commit"]),
        )
        contract.validate()
        return contract

    @classmethod
    def load(cls, path):
        with Path(path).open("r", encoding="utf-8") as stream:
            return cls.from_dict(json.load(stream))

    def validate(self):
        if self.schema_version != 1:
            raise ValueError(f"unsupported policy manifest schema: {self.schema_version}")
        if self.action_size != len(self.joint_order):
            raise ValueError("action_size and joint_order length differ")
        if len(set(self.joint_order)) != len(self.joint_order):
            raise ValueError("joint_order contains duplicates")
        if any(name not in JOINT_BY_MODEL_NAME for name in self.joint_order):
            raise ValueError("joint_order contains an unknown or passive joint")
        if any(name in PASSIVE_CLOSED_LOOP_JOINTS for name in self.joint_order):
            raise ValueError("policy must not target passive closed-loop joints")
        if len(self.action_offsets) != self.action_size or len(self.action_scales) != self.action_size:
            raise ValueError("action normalization length differs from action_size")
        if len(self.target_clips) != self.action_size:
            raise ValueError("target_clips length differs from action_size")
        if any(scale <= 0.0 for scale in self.action_scales):
            raise ValueError("action scales must be positive")
        if any(lower >= upper for lower, upper in self.target_clips):
            raise ValueError("target clip lower bound must be smaller than upper bound")
        if self.runner_action_clip <= 0.0 or self.policy_hz <= 0.0:
            raise ValueError("runner_action_clip and policy_hz must be positive")
        if self.observation_size <= 0:
            raise ValueError("observation_size must be positive")

    def to_dict(self):
        return {
            "schema_version": self.schema_version,
            "policy_file": self.policy_file,
            "policy_sha256": self.policy_sha256,
            "joint_order": list(self.joint_order),
            "action_offsets": list(self.action_offsets),
            "action_scales": list(self.action_scales),
            "target_clips": [list(bounds) for bounds in self.target_clips],
            "runner_action_clip": self.runner_action_clip,
            "observation_size": self.observation_size,
            "action_size": self.action_size,
            "policy_hz": self.policy_hz,
            "description_commit": self.description_commit,
            "common_commit": self.common_commit,
        }

    def save(self, path):
        with Path(path).open("w", encoding="utf-8") as stream:
            json.dump(self.to_dict(), stream, indent=2, ensure_ascii=False)
            stream.write("\n")

    def verify_policy(self, manifest_path):
        policy_path = Path(manifest_path).resolve().parent / self.policy_file
        if not policy_path.is_file():
            raise FileNotFoundError(policy_path)
        actual_hash = sha256_file(policy_path)
        if actual_hash != self.policy_sha256:
            raise ValueError(f"policy hash mismatch: expected {self.policy_sha256}, got {actual_hash}")
        return policy_path


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
