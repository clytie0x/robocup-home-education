"""Webcam open backoff and the single-owner lock. Importing the script must not start the server."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import webcam_view  # noqa: E402


class _Cap:
    def __init__(self, opened):
        self._opened = opened
        self.released = False

    def isOpened(self):
        return self._opened

    def release(self):
        self.released = True


def test_importing_the_script_does_not_serve():
    assert webcam_view.latest["jpg"] is None


def test_open_capture_backs_off_then_returns_the_camera():
    waits = []
    tries = {"n": 0}

    def open_fn(_index):
        tries["n"] += 1
        return _Cap(tries["n"] == 3)

    cap = webcam_view.open_capture(0, open_fn=open_fn, sleep=waits.append, delays=(0, 0.5, 1))
    assert cap.isOpened()
    assert waits == [0.5, 1]
    assert tries["n"] == 3


def test_open_capture_gives_up():
    cap = webcam_view.open_capture(0, open_fn=lambda _i: _Cap(False), sleep=lambda _d: None, delays=(0, 0.5))
    assert cap is None


def test_a_second_webcam_is_refused(tmp_path):
    first = webcam_view.claim_camera(0, tmp_path)
    try:
        try:
            webcam_view.claim_camera(0, tmp_path)
        except SystemExit as exc:
            assert "already open" in str(exc)
            assert str(os.getpid()) in str(exc)
        else:
            raise AssertionError("a second owner was allowed")
    finally:
        first.close()
