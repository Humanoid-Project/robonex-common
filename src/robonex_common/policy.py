import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from .joints import JOINT_BY_MODEL_NAME, PASSIVE_CLOSED_LOOP_JOINTS, POLICY_JOINT_ORDER
from .limits import action_normalization

ACTION_CONTRACT_TOLERANCE_RAD = 1.0e-9


@dataclass(frozen=True)
class PolicyContract:
    schema_version: int
    task: str
    policy_file: str
    policy_sha256: str
    description_sha256: str
    common_sha256: str
    training_sha256: str
    description_model: str
    joint_order: tuple[str, ...]
    observation_terms: tuple[str, ...]
    action_offsets: tuple[float, ...]
    action_scales: tuple[float, ...]
    target_clips: tuple[tuple[float, float], ...]
    runner_action_clip: float
    observation_size: int
    action_size: int
    policy_hz: float
    description_commit: str
    common_commit: str
    training_commit: str

    @classmethod
    def from_dict(cls, data):
        schema_version = int(data.get("schema_version", 0))
        if schema_version != 2:
            raise ValueError(f"unsupported policy manifest schema: {schema_version}")
        try:
            contract = cls(
                schema_version=schema_version,
                task=str(data["task"]),
                policy_file=str(data["policy_file"]),
                policy_sha256=str(data["policy_sha256"]),
                description_sha256=str(data["description_sha256"]),
                common_sha256=str(data["common_sha256"]),
                training_sha256=str(data["training_sha256"]),
                description_model=str(data["description_model"]),
                joint_order=tuple(data["joint_order"]),
                observation_terms=tuple(data["observation_terms"]),
                action_offsets=tuple(float(value) for value in data["action_offsets"]),
                action_scales=tuple(float(value) for value in data["action_scales"]),
                target_clips=tuple((float(bounds[0]), float(bounds[1])) for bounds in data["target_clips"]),
                runner_action_clip=float(data["runner_action_clip"]),
                observation_size=int(data["observation_size"]),
                action_size=int(data["action_size"]),
                policy_hz=float(data["policy_hz"]),
                description_commit=str(data["description_commit"]),
                common_commit=str(data["common_commit"]),
                training_commit=str(data["training_commit"]),
            )
        except KeyError as error:
            raise ValueError(f"policy manifest is missing field: {error.args[0]}") from error
        contract.validate()
        return contract

    @classmethod
    def load(cls, path):
        with Path(path).open("r", encoding="utf-8") as stream:
            return cls.from_dict(json.load(stream))

    def validate(self):
        if self.schema_version != 2:
            raise ValueError(f"unsupported policy manifest schema: {self.schema_version}")
        for name, value in (
            ("policy_sha256", self.policy_sha256),
            ("description_sha256", self.description_sha256),
            ("common_sha256", self.common_sha256),
            ("training_sha256", self.training_sha256),
        ):
            if len(value) != 64 or any(character not in "0123456789abcdef" for character in value.lower()):
                raise ValueError(f"{name} must be a SHA-256 digest")
        if self.action_size != len(self.joint_order):
            raise ValueError("action_size and joint_order length differ")
        if len(set(self.joint_order)) != len(self.joint_order):
            raise ValueError("joint_order contains duplicates")
        if any(name not in JOINT_BY_MODEL_NAME for name in self.joint_order):
            raise ValueError("joint_order contains an unknown or passive joint")
        if any(name in PASSIVE_CLOSED_LOOP_JOINTS for name in self.joint_order):
            raise ValueError("policy must not target passive closed-loop joints")
        if tuple(self.joint_order) != tuple(POLICY_JOINT_ORDER):
            raise ValueError(
                "joint_order must match POLICY_JOINT_ORDER exactly; "
                "a permuted order swaps legs at deploy time without any error"
            )
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
        if not self.task or not self.description_model or not self.observation_terms:
            raise ValueError("task, description_model, and observation_terms are required")
        offsets, scales, clips = action_normalization()
        for index, name in enumerate(self.joint_order):
            expected = (offsets[name], scales[name], clips[name][0], clips[name][1])
            actual = (
                self.action_offsets[index],
                self.action_scales[index],
                self.target_clips[index][0],
                self.target_clips[index][1],
            )
            if any(abs(a - b) > ACTION_CONTRACT_TOLERANCE_RAD for a, b in zip(actual, expected)):
                raise ValueError(
                    f"action contract mismatch for {name}: "
                    f"manifest offset/scale/clip={actual}, robonex-common={expected}. "
                    "The policy was trained against a different joint definition, so its "
                    "actions map to different targets now. Retrain and export a new manifest."
                )

    def to_dict(self):
        return {
            "schema_version": self.schema_version,
            "task": self.task,
            "policy_file": self.policy_file,
            "policy_sha256": self.policy_sha256,
            "description_sha256": self.description_sha256,
            "common_sha256": self.common_sha256,
            "training_sha256": self.training_sha256,
            "description_model": self.description_model,
            "joint_order": list(self.joint_order),
            "observation_terms": list(self.observation_terms),
            "action_offsets": list(self.action_offsets),
            "action_scales": list(self.action_scales),
            "target_clips": [list(bounds) for bounds in self.target_clips],
            "runner_action_clip": self.runner_action_clip,
            "observation_size": self.observation_size,
            "action_size": self.action_size,
            "policy_hz": self.policy_hz,
            "description_commit": self.description_commit,
            "common_commit": self.common_commit,
            "training_commit": self.training_commit,
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


def sha256_paths(paths, root):
    root = Path(root).resolve()
    resolved = sorted({Path(path).resolve() for path in paths}, key=lambda path: path.as_posix())
    if not resolved:
        raise ValueError("no files selected for SHA-256")
    digest = hashlib.sha256()
    for path in resolved:
        if not path.is_file():
            raise FileNotFoundError(path)
        relative = path.relative_to(root).as_posix().encode("utf-8")
        content = path.read_bytes()
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(len(content).to_bytes(8, "big"))
        digest.update(content)
    return digest.hexdigest()


def python_source_sha256(repo_root, relative_roots):
    repo_root = Path(repo_root).resolve()
    paths = []
    for relative_root in relative_roots:
        source = repo_root / relative_root
        if source.is_file() and source.suffix == ".py":
            paths.append(source)
        elif source.is_dir():
            paths.extend(source.rglob("*.py"))
    return sha256_paths(paths, repo_root)


def mujoco_bundle_sha256(description_root, description_model):
    description_root = Path(description_root).resolve()
    pending = [(description_root / description_model).resolve()]
    visited = set()
    assets = set()
    while pending:
        xml_path = pending.pop()
        if xml_path in visited:
            continue
        if not xml_path.is_file():
            raise FileNotFoundError(xml_path)
        visited.add(xml_path)
        root = ET.parse(xml_path).getroot()
        compiler = root.find("compiler")
        mesh_dir = xml_path.parent
        texture_dir = xml_path.parent
        if compiler is not None:
            if compiler.get("meshdir"):
                mesh_dir = (xml_path.parent / compiler.get("meshdir")).resolve()
            if compiler.get("texturedir"):
                texture_dir = (xml_path.parent / compiler.get("texturedir")).resolve()
        for include in root.iter("include"):
            pending.append((xml_path.parent / include.attrib["file"]).resolve())
        for element in root.iter():
            file_name = element.get("file")
            if not file_name or element.tag == "include":
                continue
            asset_dir = texture_dir if element.tag == "texture" else mesh_dir
            assets.add((asset_dir / file_name).resolve())
    return sha256_paths(visited | assets, description_root)
