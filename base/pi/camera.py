"""Colour and depth frames from the Xtion through OpenNI2, on a background thread.

Venue lighting: the colour stream's auto exposure and auto white balance are turned on when it starts and read
back (auto_light); Camera.settings keeps what the driver answered. On the robot: python3 camera.py prints them
with the colour frame's mean level (the check of lane light).
"""

import ctypes as C
import inspect
import json
import threading
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    time: float  # time.monotonic() when the pair was complete
    width: int
    height: int
    rgb: bytes  # width * height * 3, RGB order
    depth: np.ndarray  # uint16 millimetres, shape (height, width), 0 = no reading


class Camera:
    """Keeps the camera open and hands out the newest frame, each one at most once."""

    def __init__(self, capture=None, clock=time.monotonic):
        self._capture = capture or capture_color_and_depth
        self._clock = clock
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._frame = None
        self._thread = None
        self.error = None
        self.settings = None  # {"auto_exposure", "auto_white_balance"}: True, False or None (no answer)

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _on_pair(self, color, depth):
        if (color["width"], color["height"]) != (depth["width"], depth["height"]):
            raise ValueError("colour and depth sizes differ: %r" % ((color["width"], color["height"]),))
        frame = Frame(self._clock(), depth["width"], depth["height"], color["rgb"], depth["mm"])
        with self._lock:
            self._frame = frame

    def _keep_settings(self, settings):
        self.settings = dict(settings)

    def _run(self):
        try:
            takes = "on_settings" in inspect.signature(self._capture).parameters  # test captures may not
            extra = {"on_settings": self._keep_settings} if takes else {}
            self._capture(365 * 24 * 3600, self._on_pair, self._stop.is_set, **extra)
            if not self._stop.is_set():
                raise RuntimeError("camera stream ended")
        except Exception as error:
            self.error = error

    def latest(self):
        with self._lock:
            frame, self._frame = self._frame, None
        return frame

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(5)


AUTO_WHITE_BALANCE, AUTO_EXPOSURE = 100, 101  # ONI_STREAM_PROPERTY_* in OpenNI2's OniProperties.h, OniBool (int)


def auto_light(library, stream):
    """Turn the colour stream's auto exposure and auto white balance on, then read both back.

    Returns {"auto_exposure": ..., "auto_white_balance": ...}, each True or False as the driver reads it back,
    or None when it does not answer (then nothing is known: check on the robot with python3 camera.py).
    """
    found = {}
    for name, prop in (("auto_exposure", AUTO_EXPOSURE), ("auto_white_balance", AUTO_WHITE_BALANCE)):
        on = C.c_int(1)
        library.oniStreamSetProperty(stream, prop, C.byref(on), C.sizeof(on))
        value, size = C.c_int(-1), C.c_int(C.sizeof(C.c_int))
        if library.oniStreamGetProperty(stream, prop, C.byref(value), C.byref(size)) != 0:
            found[name] = None
        else:
            found[name] = bool(value.value)
    return found


# Taken from xohm@pi:~/pi-drive/person.py:_capture_color_and_depth on 2026-11-13; depth is now a
# numpy array (frombuffer) instead of a struct.unpack tuple.
def capture_color_and_depth(seconds, on_pair=None, should_stop=None, on_settings=None):
    """on_settings(dict), when given, gets auto_light's answer for the colour stream once it started."""
    library = C.CDLL("libOpenNI2.so.0")

    class DevInfo(C.Structure):
        _fields_ = [
            ("uri", C.c_char * 256),
            ("vendor", C.c_char * 256),
            ("name", C.c_char * 256),
            ("vid", C.c_uint16),
            ("pid", C.c_uint16),
        ]

    class VMode(C.Structure):
        _fields_ = [("fmt", C.c_int), ("x", C.c_int), ("y", C.c_int), ("fps", C.c_int)]

    class OniFrame(C.Structure):
        _fields_ = [
            ("dataSize", C.c_int),
            ("data", C.c_void_p),
            ("sensor", C.c_int),
            ("ts", C.c_uint64),
            ("idx", C.c_int),
            ("w", C.c_int),
            ("h", C.c_int),
            ("vm", VMode),
            ("crop", C.c_int),
            ("cx", C.c_int),
            ("cy", C.c_int),
            ("stride", C.c_int),
        ]

    device = C.c_void_p()
    handles = []
    if library.oniInitialize(2002) != 0:
        raise RuntimeError("openni_init_failed")
    try:
        listed = C.POINTER(DevInfo)()
        count = C.c_int()
        library.oniGetDeviceList(C.byref(listed), C.byref(count))
        if count.value < 1 or library.oniDeviceOpen(listed[0].uri, C.byref(device)) != 0:
            raise RuntimeError("camera_open_failed")
        for sensor in (2, 3):
            handle = C.c_void_p()
            if library.oniDeviceCreateStream(device, sensor, C.byref(handle)) != 0 or library.oniStreamStart(handle) != 0:
                raise RuntimeError("stream_failed")
            handles.append(handle)
        library.oniStreamSetProperty.argtypes = [C.c_void_p, C.c_int, C.c_void_p, C.c_int]
        library.oniStreamGetProperty.argtypes = [C.c_void_p, C.c_int, C.c_void_p, C.POINTER(C.c_int)]
        settings = auto_light(library, handles[0])  # sensor 2: colour
        if on_settings is not None:
            on_settings(settings)
        color = depth = None
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not (should_stop and should_stop()) and (color is None or depth is None):
            for handle in handles:
                ready = (C.c_void_p * 1)(handle)
                which = C.c_int()
                if library.oniWaitForAnyStream(ready, 1, C.byref(which), 200) != 0:
                    continue
                pointer = C.POINTER(OniFrame)()
                if library.oniStreamReadFrame(handle, C.byref(pointer)) != 0:
                    continue
                frame = pointer.contents
                if frame.vm.fmt == 200 and color is None:
                    color = {
                        "width": frame.w,
                        "height": frame.h,
                        "rgb": C.string_at(frame.data, frame.w * frame.h * 3),
                    }
                elif frame.vm.fmt == 100 and depth is None:
                    # string_at copies, so the array stays valid after oniFrameRelease.
                    raw = C.string_at(frame.data, frame.w * frame.h * 2)
                    depth = {
                        "width": frame.w,
                        "height": frame.h,
                        "mm": np.frombuffer(raw, dtype="<u2").reshape(frame.h, frame.w),
                    }
                library.oniFrameRelease(pointer)
            if color is not None and depth is not None:
                if on_pair is None:
                    return color, depth
                on_pair(color, depth)
                color = depth = None
        return None if color is None or depth is None else (color, depth)
    finally:
        for handle in handles:
            library.oniStreamStop(handle)
            library.oniStreamDestroy(handle)
        if device.value:
            library.oniDeviceClose(device)
        library.oniShutdown()


def main():
    """On the robot: open the Xtion, print the colour stream's auto exposure and white balance as the driver reads
    them back, and the mean level of one colour frame."""
    import numpy as np

    got = {}
    pair = capture_color_and_depth(5.0, on_settings=got.update)
    rgb = None if pair is None else np.frombuffer(pair[0]["rgb"], np.uint8)
    print(json.dumps(dict(got, rgb_mean=None if rgb is None else round(float(rgb.mean()), 1))))


if __name__ == "__main__":
    main()
