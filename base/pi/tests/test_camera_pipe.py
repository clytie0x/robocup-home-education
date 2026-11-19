"""The console preview is slower and smaller. Follow's camera is a different process."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import camera_pipe  # noqa: E402
import run_follow  # noqa: E402

CAMERA = Path(__file__).resolve().parent.parent / "camera.py"


class Preview(unittest.TestCase):
    def test_the_console_waits_out_an_8_hz_period(self):
        self.assertEqual(camera_pipe.CONSOLE_JPEG_QUALITY, 50)
        self.assertEqual(camera_pipe.CONSOLE_FPS, 8.0)
        self.assertIsNone(camera_pipe.next_encode_at(1.0, 1.05))
        nxt = camera_pipe.next_encode_at(1.0, 0.0)
        self.assertAlmostEqual(nxt, 1.0 + 1.0 / 8.0)
        self.assertIsNone(camera_pipe.next_encode_at(nxt - 0.01, nxt))
        self.assertAlmostEqual(camera_pipe.next_encode_at(nxt, nxt), nxt + 0.125)

    def test_follow_capture_is_not_capped_by_the_preview(self):
        self.assertNotIn("CONSOLE_FPS", CAMERA.read_text(encoding="utf-8"))
        self.assertEqual(run_follow.PERIOD, 0.1)
        src = Path(camera_pipe.__file__).read_text(encoding="utf-8")
        self.assertIn('struct.pack(">I"', src)


if __name__ == "__main__":
    unittest.main()
