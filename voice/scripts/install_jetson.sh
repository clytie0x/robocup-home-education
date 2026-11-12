#!/usr/bin/env bash
# One-time setup on Jetson Orin Nano (JetPack 7.2 / Ubuntu 24.04 / CUDA 13 / ROS 2 Jazzy)
# Builds whisper.cpp + llama.cpp with CUDA (sm_87) and downloads small English models.
set -euo pipefail

ROOT=/opt/robot_voice
sudo mkdir -p $ROOT && sudo chown -R "$USER" $ROOT
mkdir -p $ROOT/models/{whisper,llm,piper} $ROOT/src

sudo apt update
sudo apt install -y git cmake build-essential curl libportaudio2 portaudio19-dev \
                    python3-pip python3-numpy python3-requests alsa-utils

# CUDA on PATH (JetPack installs it under /usr/local/cuda)
export PATH=/usr/local/cuda/bin:$PATH
nvcc --version

# ---------- whisper.cpp (STT) ----------
cd $ROOT/src
[ -d whisper.cpp ] || git clone --depth 1 https://github.com/ggml-org/whisper.cpp
cd whisper.cpp
cmake -B build -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87 -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)" --target whisper-server
bash ./models/download-ggml-model.sh base.en-q5_1 $ROOT/models/whisper
bash ./models/download-ggml-model.sh small.en-q5_1 $ROOT/models/whisper   # more accurate, used automatically

# ---------- llama.cpp (LLM) ----------
cd $ROOT/src
[ -d llama.cpp ] || git clone --depth 1 https://github.com/ggml-org/llama.cpp
cd llama.cpp
cmake -B build -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87 -DCMAKE_BUILD_TYPE=Release -DLLAMA_CURL=OFF
cmake --build build -j"$(nproc)" --target llama-server

HF=https://huggingface.co
curl -L -o $ROOT/models/llm/qwen2.5-1.5b-instruct-q4_k_m.gguf \
  $HF/Qwen/Qwen2.5-1.5B-Instruct-GGUF/resolve/main/qwen2.5-1.5b-instruct-q4_k_m.gguf

# ---------- Piper (TTS, CPU) ----------
pip3 install --break-system-packages piper-tts sounddevice webrtcvad-wheels
V=$HF/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium
curl -L -o $ROOT/models/piper/en_US-lessac-medium.onnx      $V/en_US-lessac-medium.onnx
curl -L -o $ROOT/models/piper/en_US-lessac-medium.onnx.json $V/en_US-lessac-medium.onnx.json

echo "Done. Next: scripts/start_servers.sh, then build the ROS 2 package."
