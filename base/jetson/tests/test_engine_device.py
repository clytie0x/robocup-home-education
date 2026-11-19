"""TensorRT device-mismatch detection, and re-id giving up cleanly when execute fails."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import engine  # noqa: E402
import reid  # noqa: E402


def test_device_mismatch_reads_the_tensorrt_warning():
    assert engine.device_mismatch(["engine plan file is generated on an incompatible device"])
    assert engine.device_mismatch(["WARNING: The engine plan file is built for a different device"])
    assert not engine.device_mismatch(["loaded"])
    assert not engine.device_mismatch([])


def test_the_execute_error_still_contains_the_old_words():
    err = engine.EngineError(
        "TensorRT execute failed. This engine was built for a different device: rebuild it on this Jetson.",
        mismatch=True)
    assert err.mismatch
    assert "TensorRT execute failed" in str(err)


def test_a_failed_reid_returns_no_look_instead_of_raising(tmp_path):
    failed = reid.ReId.__new__(reid.ReId)

    class Boom:
        def run(self, _batch):
            raise engine.EngineError("TensorRT execute failed", mismatch=True)

    failed._engine = Boom()
    failed._max_batch = 4
    failed._EngineError = engine.EngineError
    failed._fallback_path = str(tmp_path / "missing.engine")
    failed._gave_up = False
    image = np.zeros((32, 32, 3), np.uint8)
    assert failed.looks(image, [[0, 0, 16, 16]]) == [None]
    assert failed.looks(image, []) == []
