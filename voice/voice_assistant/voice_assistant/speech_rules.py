"""stt_node's pure parts: the transcript filter, the volume gate and the barge-in detector.

No ROS, no audio device, numpy only, so they can be unit-tested on a PC. stt_node.py keeps the capture,
the VAD loop and the whisper request.
"""
import collections
import math
import re

import numpy as np

BARGE_RX = re.compile(r"\b(stop|halt|freeze)\b|^\W*(?:robot\W+|please\W+)*(finish(?:ed)?|pause|end)\W*$", re.I)   # finish/pause/end only as the whole sentence


def frame_rms(frame: np.ndarray) -> float:
    """RMS of one audio frame. An empty frame is 0, not NaN (NaN would poison the noise floor)."""
    if frame is None or len(frame) == 0:
        return 0.0
    value = float(np.sqrt(np.mean(np.square(frame.astype(np.float32)))))
    return value if math.isfinite(value) else 0.0


def _real(value):
    """A finite real number, or None. Rejects bools, text, and NaN/inf."""
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def whisper_text(payload) -> str:
    """The transcript string from a whisper.cpp JSON body, or "" when the body is not that shape.

    A 200 response of `null`, `[]`, or `{"text": null}` is silence, not a failed request.
    """
    if not isinstance(payload, dict):
        return ""
    text = payload.get("text", "")
    if not isinstance(text, str):
        return ""
    return text.strip()


def noise_transcript(text, allow):
    """True when whisper's transcript is not worth passing on: empty, bracketed ("[BLANK_AUDIO]"),
    shorter than 2 characters, or ONE word that is not in `allow` (whisper invents "Go." / "You." from
    noise and its own echo). `allow` empty = keep every transcript. Longer sentences always pass: the
    flow decides what they mean."""
    t = (text or "").strip()
    if not t or t.startswith(("[", "(")) or len(t) < 2:
        return True
    if allow:
        words = re.findall(r"[a-z']+", t.lower())
        if len(words) == 1 and words[0] not in allow:
            return True
    return False


class LoudnessGate:
    """Tracks the background noise level and decides whether a clip is loud enough to be speech
    aimed at the robot. Level of a clip = 75th percentile of its 30 ms frame RMS values."""

    def __init__(self, min_snr=2.0, min_rms=0.0, alpha=0.05, floor_min=30.0):
        self.min_snr, self.min_rms = min_snr, min_rms
        self.alpha, self.floor_min = alpha, floor_min
        self.noise = None

    def update_noise(self, rms: float):
        """Call with frames that are NOT speech (robot not talking).

        Non-finite samples are skipped. One NaN would otherwise make every later clip look silent.
        """
        number = _real(rms)
        if number is None or number < 0:
            return
        self.noise = number if self.noise is None else (1 - self.alpha) * self.noise + self.alpha * number

    def check(self, frame_levels):
        clean = []
        for sample in frame_levels or []:
            number = _real(sample)
            if number is not None:
                clean.append(number)
        level = float(np.percentile(clean, 75)) if clean else 0.0
        noise = _real(self.noise)
        noise = self.floor_min if noise is None else max(noise, self.floor_min)
        if noise <= 0:
            noise = 1e-6
        snr = level / noise
        ok = level >= self.min_rms and (self.min_snr <= 0 or snr >= self.min_snr)
        return ok, level, noise, snr


class BargeInDetector:
    """Finds the user's voice on top of the robot's own voice while the robot is talking.
    Echo level = 90th percentile of the last ~2 s of frame RMS (frames judged to be the robot only).
    A frame is "loud" when it is `ratio` times above that. 3 loud frames start a segment, 300 ms below
    the threshold (or 2 s total) end it. feed() returns the segment's audio when it ends, else None."""

    def __init__(self, ratio=2.0, min_level=0.0, window=70, start_frames=3, end_frames=10,
                 max_frames=66, min_loud=5, pre_roll=6):
        self.ratio, self.min_level = ratio, min_level
        self.extra = 1.0                    # set > 1 while the robot itself is saying "stop"
        self.echo = collections.deque(maxlen=window)
        self.start_frames, self.end_frames = start_frames, end_frames
        self.max_frames, self.min_loud, self.pre_n = max_frames, min_loud, pre_roll
        self.reset()

    def reset(self):
        self.echo.clear()
        self.pre, self.seg = [], []
        self.loud_run = self.quiet_run = self.n_loud = 0
        self.active = False

    def threshold(self):
        if len(self.echo) < 10:             # not enough robot audio heard yet
            return None
        return max(float(np.percentile(self.echo, 90)) * self.ratio * self.extra, self.min_level)

    def feed(self, frame, rms):
        number = _real(rms)
        rms = 0.0 if number is None or number < 0 else number
        thr = self.threshold()
        loud = thr is not None and rms > thr
        if not self.active:
            if not loud:
                self.echo.append(rms)       # learn the robot's echo level from non-loud frames only
            self.pre = (self.pre + [frame])[-self.pre_n:]
            self.loud_run = self.loud_run + 1 if loud else 0
            if self.loud_run >= self.start_frames:
                self.active, self.seg = True, list(self.pre)
                self.n_loud, self.quiet_run = self.loud_run, 0
            return None
        self.seg.append(frame)
        if loud:
            self.n_loud += 1
            self.quiet_run = 0
        else:
            self.quiet_run += 1
            self.echo.append(rms)
        if self.quiet_run >= self.end_frames or len(self.seg) >= self.max_frames:
            seg, n_loud = self.seg, self.n_loud
            self.active, self.seg, self.pre = False, [], []
            self.loud_run = self.quiet_run = self.n_loud = 0
            return np.concatenate(seg) if n_loud >= self.min_loud else None
        return None
