"""TTS node: /tts/say -> Piper (CPU, ~60-120 MB RAM) -> speaker
          /tts/play (path to a 16-bit .wav) -> plays the clip (e.g. the beatbox)

Publishes /tts/speaking (Bool) so the STT node mutes the mic while the robot talks.
/tts/interrupt (Empty) stops the current speech / clip at once and drops everything queued
(sent by stt_node when the user says "stop" over the robot).
Output is adapted to what the speaker accepts: if it refuses the voice's 22050 Hz or mono,
the audio is resampled to the device's own rate and/or duplicated to stereo.
Works with both the new piper-tts API (>=1.3, `synthesize()` yields AudioChunk)
and the older one (`synthesize_stream_raw()`).
"""
import os
import queue
import re
import threading
import wave

import numpy as np
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from piper import PiperVoice


def output_devices_text() -> str:
    try:
        devs = sd.query_devices()
    except Exception as e:  # noqa: BLE001
        return f'(could not list devices: {e})'
    lines = [f'  {i}: {d["name"]}  ({d["max_output_channels"]} out, {int(d["default_samplerate"])} Hz)'
             for i, d in enumerate(devs) if d['max_output_channels'] > 0]
    return '\n'.join(lines) or '  (no output devices found - is the speaker plugged in?)'


def resample(audio: np.ndarray, src: int, dst: int) -> np.ndarray:
    """Linear resample int16 [frames, channels]."""
    if src == dst:
        return audio
    n_out = int(len(audio) * dst / src)
    xi = np.linspace(0, len(audio) - 1, n_out)
    out = np.stack([np.interp(xi, np.arange(len(audio)), audio[:, c].astype(np.float32))
                    for c in range(audio.shape[1])], axis=1)
    return np.clip(out, -32768, 32767).astype(np.int16)


class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.declare_parameter('voice_model', '/opt/robot_voice/models/piper/en_US-lessac-medium.onnx')
        self.declare_parameter('speaker_device', '')   # '' = default output
        self.declare_parameter('volume', 1.0)

        p = lambda n: self.get_parameter(n).value
        self.voice = PiperVoice.load(p('voice_model'))
        self.rate = int(self.voice.config.sample_rate)
        dev = p('speaker_device')
        self.device = int(dev) if str(dev).isdigit() else (dev or None)
        self.volume = float(p('volume'))

        self.pub_speaking = self.create_publisher(Bool, '/tts/speaking', 10)
        self.create_subscription(String, '/tts/say', self._on_say, 10)
        self.create_subscription(String, '/tts/play', self._on_play, 10)   # path to a .wav clip
        self.create_subscription(Empty, '/tts/interrupt', self._on_interrupt, 10)
        self.q: queue.Queue = queue.Queue()
        self.interrupted = threading.Event()
        threading.Thread(target=self._worker, daemon=True).start()
        try:
            out = sd.query_devices(self.device, 'output')
            self.get_logger().info(f'TTS ready ({p("voice_model")}, {self.rate} Hz) -> speaker: "{out["name"]}" '
                                   f'({int(out["default_samplerate"])} Hz, {out["max_output_channels"]} ch)')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'speaker_device={self.device!r} is not an output: {e}\n'
                                    f'Output devices you can use for speaker_device:\n{output_devices_text()}')

    def _on_say(self, msg: String):
        text = msg.data.strip()
        if text:
            self.q.put(('say', text))

    def _drain(self):
        try:
            while True:
                self.q.get_nowait()
        except queue.Empty:
            pass

    def _on_interrupt(self, _):
        self.interrupted.set()
        self._drain()
        sd.stop()                       # cuts the sd.play() running in the worker thread
        self.get_logger().info('interrupted by the user')

    def _on_play(self, msg: String):
        path = os.path.expanduser(msg.data.strip())
        if path:
            self.q.put(('play', path))

    def _load_wav(self, path):
        """16-bit PCM .wav -> (int16 array [frames, channels], sample_rate)."""
        with wave.open(path, 'rb') as w:
            if w.getsampwidth() != 2:
                raise ValueError(f'{path}: only 16-bit PCM wav is supported')
            ch, rate = w.getnchannels(), w.getframerate()
            audio = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).reshape(-1, ch)
        if self.volume != 1.0:
            audio = np.clip(audio * self.volume, -32768, 32767).astype(np.int16)
        return audio, rate

    def _synth(self, sentence: str) -> np.ndarray:
        if hasattr(self.voice, 'synthesize_stream_raw'):        # piper-tts <= 1.2
            raw = b''.join(self.voice.synthesize_stream_raw(sentence))
        else:                                                     # piper-tts >= 1.3
            raw = b''.join(c.audio_int16_bytes for c in self.voice.synthesize(sentence))
        audio = np.frombuffer(raw, dtype=np.int16)
        if self.volume != 1.0:
            audio = np.clip(audio * self.volume, -32768, 32767).astype(np.int16)
        return audio

    def _play(self, audio: np.ndarray, rate: int):
        """Play int16 audio ([frames] or [frames, ch]), adapting rate / channels to the speaker."""
        if audio.ndim == 1:
            audio = audio[:, None]
        ch = audio.shape[1]
        try:
            sd.check_output_settings(device=self.device, channels=ch, samplerate=rate, dtype='int16')
        except Exception:  # noqa: BLE001
            info = sd.query_devices(self.device, 'output')
            dev_rate = int(info['default_samplerate'])
            dev_ch = max(ch, min(2, int(info['max_output_channels'])))
            for r, c in [(rate, dev_ch), (dev_rate, ch), (dev_rate, dev_ch), (48000, dev_ch), (44100, dev_ch)]:
                try:
                    sd.check_output_settings(device=self.device, channels=c, samplerate=r, dtype='int16')
                    break
                except Exception:  # noqa: BLE001
                    continue
            else:
                raise RuntimeError(f'speaker "{info["name"]}" accepts none of the tried rates/channels')
            if c != ch:
                audio = np.repeat(audio[:, :1], c, axis=1)
            audio, rate = resample(audio, rate, r), r
            self.get_logger().info(f'speaker needs {r} Hz / {c} ch -> converting', once=True)
        sd.play(audio, rate, device=self.device, blocking=True)

    def _worker(self):
        while rclpy.ok():
            try:
                kind, item = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            self.interrupted.clear()
            self.pub_speaking.publish(Bool(data=True))   # mutes the mic (also while a clip plays)
            try:
                if kind == 'play':
                    audio, rate = self._load_wav(item)
                    self.get_logger().info(f'playing {item} ({len(audio) / rate:.1f}s)')
                    self._play(audio, rate)
                else:
                    # speak sentence by sentence -> first audio comes out sooner
                    for s in re.split(r'(?<=[.!?])\s+', item):
                        if self.interrupted.is_set():
                            break
                        if s.strip():
                            self._play(self._synth(s), self.rate)
            except Exception as e:
                self.get_logger().error(f'TTS/playback failed: {e}\n'
                                        f'Output devices you can use for speaker_device:\n{output_devices_text()}')
            finally:
                if self.interrupted.is_set():
                    self._drain()
                    self.interrupted.clear()
                if self.q.empty():
                    self.pub_speaking.publish(Bool(data=False))


def main():
    rclpy.init()
    node = TtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
