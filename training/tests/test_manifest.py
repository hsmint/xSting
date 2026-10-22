import copy
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("validate_model", ROOT / "validate_model.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class ManifestTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.model = Path(self.directory.name) / "fixture.bin"
        self.model.write_bytes(b"test fixture, not an ONNX model")
        self.m = dict(schema_version=1, model_version="test", joint_names=["j1", "j2"],
            action_mode="segment_delta_position", action_units="rad", observation_dim=3,
            action_dim=2, observation_mean=[0, 0, 0], observation_scale=[1, 1, 1], action_scale=[0.1, 0.1],
            sha256=hashlib.sha256(self.model.read_bytes()).hexdigest())

    def test_valid_metadata_and_integrity(self):
        self.assertTrue(mod.validate(self.m, self.model))

    def test_corrupt_model(self):
        self.model.write_bytes(b"changed")
        with self.assertRaises(ValueError): mod.validate(self.m, self.model)

    def test_missing_model(self):
        self.model.unlink()
        with self.assertRaises(OSError): mod.validate(self.m, self.model)

    def test_bad_contracts(self):
        cases = [("joint_names", ["j1", "j1"]), ("action_mode", "torque_20hz"),
            ("action_units", "degrees"), ("observation_dim", 0), ("action_dim", 9),
            ("observation_scale", [1, 0, 1]), ("observation_mean", [0, float("nan"), 0]),
            ("action_scale", [0.1]), ("model_version", ""), ("schema_version", 2),
            ("schema_version", True), ("action_dim", True)]
        for key, value in cases:
            with self.subTest(key=key):
                m = copy.deepcopy(self.m);m[key] = value
                with self.assertRaises(ValueError): mod.validate(m, self.model)

    def test_non_object_manifest(self):
        with self.assertRaises(ValueError): mod.validate([], self.model)


if __name__ == "__main__": unittest.main()
