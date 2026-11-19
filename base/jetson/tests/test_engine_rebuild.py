"""An interrupted trtexec must not replace the live engine, and a 0-byte plan is missing."""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import engine  # noqa: E402
import server  # noqa: E402


def _save_engine(cmd):
    return next(part.split("=", 1)[1] for part in cmd if str(part).startswith("--saveEngine="))


class RebuildKeepsTheLiveEngine(unittest.TestCase):
    def test_an_interrupted_build_leaves_the_live_file_unchanged(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        live = os.path.join(folder.name, "osnet_ain_x1_0_msmt17_256x128.engine")
        original = b"live-plan-bytes"
        with open(live, "wb") as handle:
            handle.write(original)

        def interrupt(cmd):
            dest = _save_engine(cmd)
            self.assertNotEqual(os.path.abspath(dest), os.path.abspath(live))
            with open(dest, "wb") as handle:
                handle.write(b"")  # trtexec was killed while writing the temp plan
            return 1

        self.assertFalse(engine.rebuild_static(os.path.join(folder.name, "a.onnx"), live,
                                                run=interrupt, free_bytes=0))
        with open(live, "rb") as handle:
            self.assertEqual(handle.read(), original)
        self.assertEqual(os.listdir(folder.name), [os.path.basename(live)])

        def empty_but_zero(cmd):
            with open(_save_engine(cmd), "wb") as handle:
                handle.write(b"")
            return 0

        self.assertFalse(engine.rebuild_static(os.path.join(folder.name, "a.onnx"), live,
                                                run=empty_but_zero, free_bytes=0))
        with open(live, "rb") as handle:
            self.assertEqual(handle.read(), original)
        self.assertEqual(os.listdir(folder.name), [os.path.basename(live)])

    def test_a_zero_byte_engine_is_missing_and_does_not_load(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        empty = os.path.join(folder.name, "osnet_ain_x1_0_msmt17_256x128.engine")
        good = os.path.join(folder.name, "osnet_x0_25_msmt17_256x128.engine")
        open(empty, "wb").close()
        with open(good, "wb") as handle:
            handle.write(b"plan")
        self.assertFalse(engine.engine_present(empty))
        self.assertFalse(engine.engine_present(os.path.join(folder.name, "absent.engine")))
        self.assertTrue(engine.engine_present(good))
        with self.assertRaises(engine.EngineError):
            engine.Engine(empty)
        self.assertEqual(server._pick_engine(empty, good), good)
        self.assertEqual(server._pick_engine(good, empty), good)
        self.assertEqual(server._pick_engine(empty, os.path.join(folder.name, "absent.engine")), empty)


if __name__ == "__main__":
    unittest.main()
