"""STT node: microphone -> VAD -> whisper.cpp server -> /speech/heard (-> dialog_node -> /speech/text)

- Records from the microphone with sounddevice
- Uses WebRTC VAD to cut one utterance (speech followed by silence)
- Volume gate: drops clips that are not clearly louder than the room's background noise
  (far-away chatter, fans), so whisper does not turn them into words
- Opens the mic robustly: falls back to stereo / the device's own sample rate, and if the mic
  cannot be opened it lists the valid input devices and keeps retrying
- Sends the clip to whisper-server (whisper.cpp, CUDA) over HTTP
- Mutes itself while the robot is speaking (/tts/speaking) so it does not hear itself
- Barge-in: while the robot speaks it still listens for sounds clearly LOUDER than the robot's own
  voice at the mic; if whisper hears "stop" / "halt" / "freeze" in them it publishes /tts/interrupt
  (robot stops talking at once) and sends "stop" on as a normal sentence
"""
import io
import collections
import queue
import re
import threading
import time
import wave

import numpy as np
import requests
import sounddevice as sd
import webrtcvad
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

VAD_RATE = 16000          # whisper + webrtcvad expect 16 kHz mono
FRAME_MS = 30             # webrtcvad accepts 10/20/30 ms frames
FRAME_SAMPLES = VAD_RATE * FRAME_MS // 1000


def frame_rms(frame: np.ndarray) -> float:
    return float(np.sqrt(np.mean(frame.astype(np.float32) ** 2)))


class LoudnessGate:
    """Tracks the background noise level and decides whether a clip is loud enough to be speech
    aimed at the robot. Level of a clip = 75th percentile of its 30 ms frame RMS values."""

    def __init__(self, min_snr=2.0, min_rms=0.0, alpha=0.05, floor_min=30.0):
        self.min_snr, self.min_rms = min_snr, min_rms
        self.alpha, self.floor_min = alpha, floor_min
        self.noise = None

    def update_noise(self, rms: float):
        """Call with frames that are NOT speech (robot not talking)."""
        self.noise = rms if self.noise is None else (1 - self.alpha) * self.noise + self.alpha * rms

    def check(self, frame_levels):
        level = float(np.percentile(frame_levels, 75)) if len(frame_levels) else 0.0
        noise = max(self.noise if self.noise is not None else self.floor_min, self.floor_min)
        snr = level / noise
        ok = level >= self.min_rms and (self.min_snr <= 0 or snr >= self.min_snr)
        return ok, level, noise, snr


BARGE_RX = re.compile(r"\b(stop|halt|freeze)\b", re.I)


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


def input_devices_text() -> str:
    try:
        devs = sd.query_devices()
    except Exception as e:  # noqa: BLE001
        return f'(could not list devices: {e})'
    lines = [f'  {i}: {d["name"]}  ({d["max_input_channels"]} in, {int(d["default_samplerate"])} Hz)'
             for i, d in enumerate(devs) if d['max_input_channels'] > 0]
    return '\n'.join(lines) or '  (no input devices found - is the USB mic plugged in?)'


class SttNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.declare_parameter('whisper_url', 'http://127.0.0.1:8080/inference')
        self.declare_parameter('mic_device', '')          # '' = default device, or name/index
        self.declare_parameter('mic_rate', 16000)         # use 48000 if the USB mic refuses 16k
        self.declare_parameter('vad_aggressiveness', 2)   # 0..3 (3 = most aggressive filtering)
        self.declare_parameter('silence_ms', 700)         # end of utterance after this much silence
        self.declare_parameter('min_speech_ms', 300)      # ignore clicks / very short noises
        self.declare_parameter('max_utterance_s', 12.0)
        self.declare_parameter('wake_word', '')           # e.g. 'robot'; '' = always listen
        self.declare_parameter('language', 'en')
        self.declare_parameter('output_topic', '/speech/heard')   # dialog_node decides what reaches the LLM
        self.declare_parameter('min_snr', 2.0)    # clip must be this many times louder than background (0 = off)
        self.declare_parameter('min_rms', 0)      # optional absolute minimum level (int16 RMS), 0 = off
        self.declare_parameter('echo_guard_s', 0.8)   # keep the mic muted this long after the robot stops talking
        self.declare_parameter('barge_in', True)      # listen for "stop" while the robot is talking
        self.declare_parameter('barge_ratio', 2.0)    # your voice must be this many times louder than the robot's echo
        # A transcript of ONE word is dropped unless it is in this list (whisper invents "Go.", "You.", "Okay."
        # from noise / speaker echo). Empty list = keep every transcript.
        self.declare_parameter('single_word_allow',
                               ['stop', 'halt', 'freeze', 'stops', 'stopped', 'stopping', 'stab', 'stob', 'stup', 'stap', 'top', 'tops', 'stuff', 'wait', 'enough', 'stay', 'hello', 'hi', 'robot', 'yes', 'no', 'thanks', 'beatbox', 'yeah', 'yep', 'yup', 'sure', 'okay', 'ok', 'nope', 'nah'])
        # Vocabulary hint for whisper: biases transcription toward robot commands.
        self.declare_parameter('initial_prompt',
                               'Hey robot. Robot voice commands: carry the bag, drop the bag, put the bag down, '
                               'follow me, stop, stay here, move forward, go back, turn left, turn right, do a beatbox.')

        p = lambda n: self.get_parameter(n).value
        self.url = p('whisper_url')
        self.req_rate = int(p('mic_rate'))
        self.mic_rate = self.req_rate
        self.silence_frames = int(p('silence_ms')) // FRAME_MS
        self.min_speech_frames = int(p('min_speech_ms')) // FRAME_MS
        self.max_frames = int(float(p('max_utterance_s')) * 1000) // FRAME_MS
        self.wake_word = str(p('wake_word')).lower().strip()
        self.language = p('language')
        self.initial_prompt = str(p('initial_prompt'))
        self.echo_guard_s = float(p('echo_guard_s'))
        self.single_word_allow = {w.lower() for w in p('single_word_allow')}
        self.vad = webrtcvad.Vad(int(p('vad_aggressiveness')))
        self.gate = LoudnessGate(float(p('min_snr')), float(p('min_rms')))
        self.barge_on = bool(p('barge_in'))
        self.barge = BargeInDetector(float(p('barge_ratio')))
        self.barge_q: queue.Queue = queue.Queue(maxsize=1)
        self.robot_texts: list = []      # what the robot is saying now (to avoid triggering on its own "stop")

        dev = p('mic_device')
        self.device = int(dev) if str(dev).isdigit() else (dev or None)

        self.pub = self.create_publisher(String, p('output_topic'), 10)
        self.create_subscription(Bool, '/tts/speaking', self._on_speaking, 10)
        self.create_subscription(String, '/tts/say', self._on_robot_say, 10)
        self.pub_interrupt = self.create_publisher(Empty, '/tts/interrupt', 10)
        self.speaking = False
        self.unmute_at = 0.0

        self.audio_q: queue.Queue = queue.Queue(maxsize=200)
        self._stop = threading.Event()
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._vad_loop, daemon=True).start()
        threading.Thread(target=self._barge_worker, daemon=True).start()
        self.get_logger().info(f'STT ready (whisper: {self.url}, barge-in "stop": '
                               f'{"on, x" + str(p("barge_ratio")) if self.barge_on else "off"})')

    # ---------- TTS feedback guard ----------
    def _on_speaking(self, msg: Bool):
        self.speaking = msg.data
        if not msg.data:
            self.unmute_at = time.time() + self.echo_guard_s   # let the speaker echo die out
            self.robot_texts = []

    def _on_robot_say(self, msg: String):
        self.robot_texts = (self.robot_texts + [msg.data])[-4:]
        # the robot itself says "stop" (e.g. "...and stop.") -> demand a bigger margin for barge-in
        self.barge.extra = 1.5 if any(BARGE_RX.search(t) for t in self.robot_texts) else 1.0

    def _muted(self):
        return self.speaking or time.time() < self.unmute_at

    # ---------- audio capture ----------
    def _pick_mic_settings(self):
        """Find (channels, rate, name) the mic accepts: mono first, then stereo; requested rate first,
        then the device's own default rate. Raises if the device has no input at all."""
        info = sd.query_devices(self.device, 'input')
        max_ch = int(info['max_input_channels'])
        if max_ch < 1:
            raise RuntimeError(f'device {self.device!r} ({info["name"]}) has no microphone input')
        rates = list(dict.fromkeys([self.req_rate, int(info['default_samplerate']), 48000, 44100, 16000]))
        for ch in dict.fromkeys([1, min(2, max_ch)]):
            for rate in rates:
                try:
                    sd.check_input_settings(device=self.device, channels=ch, samplerate=rate, dtype='int16')
                    return ch, rate, info['name']
                except Exception:  # noqa: BLE001
                    continue
        raise RuntimeError(f'{info["name"]}: no supported channels / sample rate')

    def _capture_loop(self):
        def cb(indata, frames, t, status):
            if status:
                self.get_logger().warn(str(status), throttle_duration_sec=5.0)
            try:
                self.audio_q.put_nowait(indata[:, 0].copy())   # first channel only
            except queue.Full:
                pass

        while not self._stop.is_set():
            try:
                ch, rate, name = self._pick_mic_settings()
                self.mic_rate = rate
                with sd.InputStream(samplerate=rate, channels=ch, dtype='int16',
                                    blocksize=rate * FRAME_MS // 1000, device=self.device, callback=cb):
                    self.get_logger().info(f'microphone open: "{name}" ({rate} Hz, {ch} ch)')
                    while not self._stop.is_set():
                        time.sleep(0.1)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(
                    f'cannot open microphone (mic_device={self.device!r}): {e}\n'
                    f'Input devices you can use for mic_device:\n{input_devices_text()}\n'
                    f'Fix mic_device in ~/.robot_voice_local.yaml (or remove it to use the default). '
                    f'Retrying in 5 s...')
                self._stop.wait(5.0)

    def _to_16k(self, x: np.ndarray) -> np.ndarray:
        if self.mic_rate == VAD_RATE:
            return x
        n_out = int(len(x) * VAD_RATE / self.mic_rate)
        xi = np.linspace(0, len(x) - 1, n_out)
        return np.interp(xi, np.arange(len(x)), x.astype(np.float32)).astype(np.int16)

    # ---------- VAD segmentation ----------
    def _vad_loop(self):
        buf = np.zeros(0, dtype=np.int16)
        pre_roll: list = []          # keep ~300 ms before speech starts
        speech: list = []
        levels: list = []            # RMS of each frame in the current utterance
        in_speech = False
        voiced = silence = 0
        was_muted = False

        while not self._stop.is_set():
            try:
                chunk = self.audio_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if self._muted():
                pre_roll.clear(); speech.clear()
                in_speech = False; voiced = silence = 0
                if not self.barge_on:
                    buf = np.zeros(0, dtype=np.int16)
                    continue
                buf = np.concatenate([buf, self._to_16k(chunk)])
                while len(buf) >= FRAME_SAMPLES:
                    frame, buf = buf[:FRAME_SAMPLES], buf[FRAME_SAMPLES:]
                    seg = self.barge.feed(frame, frame_rms(frame))
                    if seg is not None:
                        try:
                            self.barge_q.put_nowait(seg)
                        except queue.Full:
                            pass
                was_muted = True
                continue
            if was_muted:
                was_muted = False
                self.barge.reset()
                buf = np.zeros(0, dtype=np.int16)

            buf = np.concatenate([buf, self._to_16k(chunk)])
            while len(buf) >= FRAME_SAMPLES:
                frame, buf = buf[:FRAME_SAMPLES], buf[FRAME_SAMPLES:]
                is_speech = self.vad.is_speech(frame.tobytes(), VAD_RATE)
                rms = frame_rms(frame)

                if not in_speech:
                    pre_roll.append(frame)
                    pre_roll = pre_roll[-10:]
                    if is_speech:
                        voiced += 1
                        if voiced >= 3:                 # ~90 ms of speech -> start
                            in_speech = True
                            speech = list(pre_roll)
                            levels = []
                            silence = 0
                    else:
                        voiced = 0
                        self.gate.update_noise(rms)     # learn the room's background level
                    continue

                speech.append(frame)
                if is_speech:
                    levels.append(rms)
                silence = 0 if is_speech else silence + 1
                if silence >= self.silence_frames or len(speech) >= self.max_frames:
                    n_voiced = len(speech) - silence
                    clip = np.concatenate(speech)
                    in_speech = False; voiced = silence = 0
                    speech = []; pre_roll = []
                    if n_voiced < self.min_speech_frames:
                        continue
                    ok, level, noise, snr = self.gate.check(levels)
                    if not ok:
                        self.get_logger().info(f'(dropped, too quiet: level {level:.0f}, '
                                               f'background {noise:.0f}, x{snr:.1f} < x{self.gate.min_snr:g})')
                        continue
                    self._transcribe(clip, f'level x{snr:.1f}')

    # ---------- whisper ----------
    def _whisper(self, pcm: np.ndarray):
        wav = io.BytesIO()
        with wave.open(wav, 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(VAD_RATE)
            w.writeframes(pcm.tobytes())
        try:
            data = {'response_format': 'json', 'temperature': '0.0', 'language': self.language}
            if self.initial_prompt:
                data['prompt'] = self.initial_prompt
            r = requests.post(self.url,
                              files={'file': ('audio.wav', wav.getvalue(), 'audio/wav')},
                              data=data, timeout=30)
            r.raise_for_status()
            return r.json().get('text', '').strip()
        except Exception as e:
            self.get_logger().error(f'whisper request failed: {e}')
            return None

    def _barge_worker(self):
        """Transcribe loud segments heard while the robot talks; act only on stop / halt / freeze."""
        while not self._stop.is_set():
            try:
                seg = self.barge_q.get(timeout=0.5)
            except queue.Empty:
                continue
            text = self._whisper(seg)
            if not text:
                continue
            if BARGE_RX.search(text) and not any(text.lower().strip(' .!?') in t.lower() and len(text) > 6
                                                 for t in self.robot_texts):
                self.get_logger().info(f'BARGE-IN: "{text}" -> interrupting the robot')
                self.pub_interrupt.publish(Empty())
                self.pub.publish(String(data='stop'))
            else:
                self.get_logger().info(f'(while robot talks, ignored) "{text}"')

    def _transcribe(self, pcm: np.ndarray, info: str = ''):
        t0 = time.time()
        text = self._whisper(pcm)
        if text is None:
            return

        # whisper hallucinates on noise: drop empty / bracketed outputs like "[BLANK_AUDIO]"
        if not text or text.startswith(('[', '(')) or len(text) < 2:
            return
        words = re.findall(r"[a-z']+", text.lower())
        if self.single_word_allow and len(words) == 1 and words[0] not in self.single_word_allow:
            self.get_logger().info(f'(dropped, single-word noise) "{text}" [{len(pcm) / VAD_RATE:.1f}s clip]')
            return

        if self.wake_word:
            low = text.lower()
            idx = low.find(self.wake_word)
            if idx < 0:
                self.get_logger().info(f'(ignored, no wake word) "{text}"')
                return
            text = text[idx + len(self.wake_word):].lstrip(' ,.!?') or text

        self.get_logger().info(f'heard ({time.time() - t0:.2f}s, {info}): "{text}"')
        self.pub.publish(String(data=text))

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = SttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
