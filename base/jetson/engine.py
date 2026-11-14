"""Run one TensorRT engine: device buffers through libcudart with ctypes (no pycuda or cuda-python)."""

import ctypes

import numpy as np

_H2D, _D2H = 1, 2  # cudaMemcpyKind
_cuda = None
_logger = None  # TensorRT keeps the first logger it is given, so share one


def _cudart():
    global _cuda
    if _cuda is None:
        _cuda = ctypes.CDLL("libcudart.so")
    return _cuda


def _check(err, what):
    if err != 0:
        raise RuntimeError(f"{what} failed with CUDA error {err}")


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
        global _logger
        _logger = _logger or trt.Logger(trt.Logger.WARNING)
        with open(path, "rb") as f:
            self._engine = trt.Runtime(_logger).deserialize_cuda_engine(f.read())
        if self._engine is None:
            raise RuntimeError(f"could not load engine {path}")
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
        _check(cuda.cudaMemcpy(self._dev[name], array.ctypes.data, array.nbytes, _H2D), "cudaMemcpy in")
        if not self._context.execute_async_v3(self._stream.value):
            raise RuntimeError("TensorRT execute failed")
        _check(cuda.cudaStreamSynchronize(self._stream), "cudaStreamSynchronize")
        results = []
        for out in self.outputs:
            host = np.empty(tuple(self._context.get_tensor_shape(out)), self._dtype[out])
            _check(cuda.cudaMemcpy(host.ctypes.data, self._dev[out], host.nbytes, _D2H), "cudaMemcpy out")
            results.append(host)
        return results
