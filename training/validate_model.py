"""Validate deployment metadata/integrity; does not verify ONNX execution or policy quality."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def validate(manifest, model_path):
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be an object")
    if type(manifest.get("schema_version")) is not int or manifest["schema_version"] != 1:
        raise ValueError("schema_version must be 1")
    if not isinstance(manifest.get("model_version"), str) or not manifest["model_version"].strip():
        raise ValueError("model_version is required")
    names = manifest.get("joint_names")
    if not isinstance(names, list) or not names or any(not isinstance(x, str) or not x for x in names) or len(names) != len(set(names)):
        raise ValueError("joint_names must be unique and nonempty")
    if manifest.get("action_mode") != "segment_delta_position":
        raise ValueError("this scaffold accepts segment_delta_position, not a fixed-rate torque/velocity policy")
    if manifest.get("action_units") != "rad":
        raise ValueError("action_units must be rad")
    n = manifest.get("observation_dim")
    if type(n) is not int or n <= 0 or type(manifest.get("action_dim")) is not int or manifest["action_dim"] != len(names):
        raise ValueError("invalid input/output dimensions")
    for key, length in [("observation_mean", n), ("observation_scale", n), ("action_scale", len(names))]:
        values = manifest.get(key)
        if not isinstance(values, list) or len(values) != length or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError("invalid " + key)
        if key.endswith("scale") and any(v <= 0 for v in values):
            raise ValueError("positive scales required")
    model = Path(model_path)
    digest = hashlib.sha256()
    with model.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    if model.stat().st_size == 0 or manifest.get("sha256") != digest.hexdigest():
        raise ValueError("empty model or sha256 mismatch")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("model", type=Path)
    args = parser.parse_args()
    try:
        validate(json.loads(args.manifest.read_text()), args.model)
    except (ValueError, OSError, TypeError) as error:
        parser.exit(1, str(error) + "\n")
    print("Metadata and checksum valid; model inference and robot validation still required.")


if __name__ == "__main__":
    main()
