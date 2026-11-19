"""Run one TensorRT engine: device buffers through libcudart with ctypes (no pycuda or cuda-python).

A plan is built for one GPU. "engine built for a different device" plus "TensorRT execute failed" means
this .engine was not built on this Jetson (seen on re-id, 19 Nov 2026: the ain OSNet plan). cudaSetDevice(0)
runs before the plan is loaded and before every execute. ReId then tries the smaller fallback engine.
Rebuild on this Jetson, for example:

    trtexec --onnx=<the matching .onnx> --saveEngine=<the .engine path> --fp16
"""

import ctypes

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
    text = " ".join(str(m) for m in messages).lower()
    return "different device" in text or "incompatible device" in text


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
            mismatch = device_mismatch(_log_lines)
            extra = (" CUDA %s." % err) if err else ""
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
