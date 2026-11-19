"""Run one TensorRT engine: device buffers through libcudart with ctypes (no pycuda or cuda-python).

A plan is built for one GPU. "engine built for a different device" plus "TensorRT execute failed" means
this .engine was not built on this Jetson (seen on re-id, 19 Nov 2026: the ain OSNet plan). cudaSetDevice(0)
runs before the plan is loaded and before every execute. ReId then tries the smaller fallback engine.
Rebuild on this Jetson, for example:

    trtexec --onnx=<the matching .onnx> --saveEngine=<the .engine path> --fp16
"""

import ctypes
import os
import tempfile

import numpy as np

_H2D, _D2H = 1, 2  # cudaMemcpyKind
_cuda = None
_logger = None  # TensorRT keeps the first logger it is given, so share one
_log_lines = []  # text the shared logger has seen (device-mismatch warnings included)


class EngineError(RuntimeError):
    """execute_async_v3 returned false, or the plan would not load. str() still contains the old words.

    mismatch: the TensorRT log said the plan was built for another device.
    """

    def __init__(self, message, mismatch=False):
        super().__init__(message)
        self.mismatch = bool(mismatch)


def device_mismatch(messages):
    """True when TensorRT reported a plan built for another GPU."""
    if isinstance(messages, str):
        messages = [messages]
    text = " ".join(str(m) for m in messages).lower()
    return "different device" in text or "incompatible device" in text


def myelin_dynamic_batch(text):
    """True for the TRT 10.14+ InstanceNorm crash: Myelin __myl_Inor, CUDA 720, on a dynamic batch.

    514 times in detector.log from 19 Nov 12:04 on the Orin Nano Super (TRT 10.16.2.10) after the
    re-id plan switched to osnet_ain_x1_0. cudaSetDevice (this module) does not fix this kernel.
    """
    low = str(text).lower()
    return any(mark in low for mark in (
        "myl_inor", "instancenorm", "instance normalization", "cuda 720", "error 720", "(720)", " 720 ",
    ))


# A static batch of 1 is the workaround that thread names. Skip the rebuild below this much free RAM
# (the Orin had about 3.3 GiB free on 19 Nov; trtexec must not take the last of the shared memory).
REBUILD_FREE_BYTES = 2_500_000_000
REID_SPATIAL = (3, 256, 128)


def static_batch_flags(input_name, spatial=REID_SPATIAL, batch=1):
    """trtexec shape flags that pin the batch, so the plan is not a dynamic-batch InstanceNorm kernel."""
    spec = "%s:%d%s" % (input_name, batch, "".join("x%d" % int(d) for d in spatial))
    return ["--minShapes=%s" % spec, "--optShapes=%s" % spec, "--maxShapes=%s" % spec]


def trtexec_path():
    import shutil
    return shutil.which("trtexec")


def choose_backend(mismatch, failed, onnx_exists, trtexec):
    """tensorrt, rebuild, onnx or histogram. A bad plan is rebuilt on this device when that is possible."""
    if not mismatch and not failed:
        return "tensorrt"
    if onnx_exists and trtexec:
        return "rebuild"
    if onnx_exists:
        return "onnx"
    return "histogram"


def engine_present(path):
    """True when `path` is a non-empty file. A 0-byte plan is an interrupted trtexec, so it is missing."""
    try:
        return bool(path) and os.path.getsize(path) > 0
    except OSError:
        return False


def _plan_loads(path):
    """The temp plan is usable: non-empty, and a TensorRT engine when this process can deserialize one."""
    if not engine_present(path):
        return False
    try:
        import tensorrt as trt
    except ImportError:
        return True
    try:
        logger = _trt_logger(trt)
        with open(path, "rb") as handle:
            built = trt.Runtime(logger).deserialize_cuda_engine(handle.read())
        return built is not None
    except Exception:  # noqa: BLE001 — a bad plan must not replace the live engine
        return False


def rebuild_static(onnx_path, engine_path, run=None, input_name="input", free_bytes=None):
    """Rebuild `engine_path` with a static batch of 1.

    trtexec writes a temp file in the same directory. The live plan is replaced only after that file
    is non-empty and deserializes. A failed or interrupted build deletes the temp file and leaves
    the live plan untouched. `run`, when given, is called with the argv and returns 0 on success.
    Tests never start trtexec.
    """
    exe = trtexec_path()
    if run is None and not exe:
        return False
    if free_bytes is None and run is None:
        try:
            free_bytes = gpu_free_bytes()[0]
        except Exception:
            free_bytes = None
    if run is None and free_bytes is not None and free_bytes < REBUILD_FREE_BYTES:
        print("reid: skip static rebuild, %.1f GiB free" % (free_bytes / 2**30), flush=True)
        return False
    folder = os.path.dirname(os.path.abspath(engine_path)) or "."
    fd, temp_path = tempfile.mkstemp(
        prefix=".%s." % os.path.basename(engine_path), suffix=".part", dir=folder)
    os.close(fd)
    try:
        cmd = [exe or "trtexec", "--onnx=%s" % onnx_path, "--saveEngine=%s" % temp_path, "--fp16"]
        cmd += static_batch_flags(input_name)
        try:
            if run is not None:
                code = run(cmd)
            else:
                import subprocess
                code = subprocess.run(cmd, capture_output=True, text=True).returncode
        except Exception:  # noqa: BLE001 — trtexec did not finish; keep the engine we already have
            return False
        if code != 0 or not _plan_loads(temp_path):
            return False
        os.replace(temp_path, engine_path)
        temp_path = None
        return True
    finally:
        if temp_path and os.path.exists(temp_path):
            try:
                os.remove(temp_path)
            except OSError:
                pass


def _cudart():
    global _cuda
    if _cuda is None:
        _cuda = ctypes.CDLL("libcudart.so")
    return _cuda


def _check(err, what):
    if err != 0:
        raise RuntimeError(f"{what} failed with CUDA error {err}")


def _set_device():
    """The Jetson has one GPU. A context that was never placed on it makes execute_async_v3 return false."""
    cuda = _cudart()
    cuda.cudaSetDevice.argtypes = [ctypes.c_int]
    _check(cuda.cudaSetDevice(0), "cudaSetDevice")


def _last_cuda_error():
    cuda = _cudart()
    cuda.cudaGetLastError.restype = ctypes.c_int
    code = int(cuda.cudaGetLastError())
    if code == 0:
        return ""
    cuda.cudaGetErrorString.argtypes = [ctypes.c_int]
    cuda.cudaGetErrorString.restype = ctypes.c_char_p
    text = cuda.cudaGetErrorString(code)
    return "%s (%s)" % (code, text.decode() if text else "?")


def _trt_logger(trt):
    """One logger for the process. Its warnings are printed and kept for device_mismatch."""
    global _logger
    if _logger is not None:
        return _logger

    try:
        class _Logger(trt.ILogger):
            def __init__(self):
                trt.ILogger.__init__(self)

            def log(self, severity, msg):
                text = str(msg)
                _log_lines.append(text)
                if severity <= trt.ILogger.Severity.WARNING:
                    print("tensorrt: %s" % text, flush=True)

        _logger = _Logger()
    except Exception:  # noqa: BLE001 (an older TensorRT that will not subclass ILogger still loads)
        _logger = trt.Logger(trt.Logger.WARNING)
    return _logger


def gpu_free_bytes():
    """Free and total device memory (on the Jetson this is the shared RAM)."""
    free, total = ctypes.c_size_t(), ctypes.c_size_t()
    _check(_cudart().cudaMemGetInfo(ctypes.byref(free), ctypes.byref(total)), "cudaMemGetInfo")
    return free.value, total.value


class Engine:
    def __init__(self, path):
        if not engine_present(path):
            raise EngineError("could not load engine %s. The file is missing or empty." % path)
        import tensorrt as trt  # only on the Jetson, so the tests can import the callers anywhere

        cuda = _cudart()
        cuda.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        cuda.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        cuda.cudaFree.argtypes = [ctypes.c_void_p]
        _set_device()
        logger = _trt_logger(trt)
        with open(path, "rb") as f:
            self._engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        if self._engine is None:
            mismatch = device_mismatch(_log_lines)
            extra = " This engine was built for a different device: rebuild it on this Jetson." if mismatch else ""
            raise EngineError("could not load engine %s.%s" % (path, extra), mismatch=mismatch)
        self._context = self._engine.create_execution_context()
        self._stream = ctypes.c_void_p()
        _check(cuda.cudaStreamCreate(ctypes.byref(self._stream)), "cudaStreamCreate")
        self.inputs, self.outputs, self._dev, self._dtype = [], [], {}, {}
        names = [self._engine.get_tensor_name(i) for i in range(self._engine.num_io_tensors)]
        is_input = {n: self._engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT for n in names}
        # dynamic batch: allocate every tensor for the largest batch of profile 0
        max_batch = max([self._engine.get_tensor_profile_shape(n, 0)[2][0] for n in names
                         if is_input[n] and -1 in self._engine.get_tensor_shape(n)] or [1])
        for name in names:
            shape = [max_batch if d == -1 else d for d in self._engine.get_tensor_shape(name)]
            if is_input[name]:
                self.inputs.append(name)
            else:
                self.outputs.append(name)
            self._dtype[name] = np.dtype(trt.nptype(self._engine.get_tensor_dtype(name)))
            ptr = ctypes.c_void_p()
            _check(cuda.cudaMalloc(ctypes.byref(ptr), int(np.prod(shape)) * self._dtype[name].itemsize), "cudaMalloc")
            self._dev[name] = ptr
            self._context.set_tensor_address(name, ptr.value)
        self.output_shape = tuple(self._engine.get_tensor_shape(self.outputs[0]))
        self.input_shape = tuple(self._engine.get_tensor_shape(self.inputs[0]))
        self.max_input_shape = tuple(self._engine.get_tensor_profile_shape(self.inputs[0], 0)[2]) \
            if -1 in self.input_shape else self.input_shape

    def run(self, array):
        """One input array in, list of output arrays out (in the engine's output order)."""
        cuda = _cudart()
        name = self.inputs[0]
        array = np.ascontiguousarray(array, dtype=self._dtype[name])
        if -1 in self.input_shape:
            self._context.set_input_shape(name, array.shape)
        _set_device()
        _check(cuda.cudaMemcpy(self._dev[name], array.ctypes.data, array.nbytes, _H2D), "cudaMemcpy in")
        if not self._context.execute_async_v3(self._stream.value):
            err = _last_cuda_error()
            recent = " ".join(_log_lines[-8:])
            mismatch = device_mismatch(_log_lines)
            extra = (" CUDA %s." % err) if err else ""
            if myelin_dynamic_batch(recent + " " + err):
                extra += " Myelin __myl_Inor CUDA 720 (dynamic batch InstanceNorm)."
            if mismatch:
                extra += " This engine was built for a different device: rebuild it on this Jetson."
            raise EngineError("TensorRT execute failed." + extra, mismatch=mismatch)
        _check(cuda.cudaStreamSynchronize(self._stream), "cudaStreamSynchronize")
        results = []
        for out in self.outputs:
            host = np.empty(tuple(self._context.get_tensor_shape(out)), self._dtype[out])
            _check(cuda.cudaMemcpy(host.ctypes.data, self._dev[out], host.nbytes, _D2H), "cudaMemcpy out")
            results.append(host)
        return results
