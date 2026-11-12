#!/usr/bin/env bash
# Start whisper-server (:8080) and llama-server (:8081). Logs go to /tmp/robot_voice_*.log
# RAM budget (unified CPU+GPU memory on Jetson):
#   whisper base.en q5_1  ~ 0.3 GB
#   Qwen2.5-1.5B Q4_K_M, ctx 2048, 1 slot  ~ 1.4 GB
set -euo pipefail
ROOT=/opt/robot_voice
LLM_MODEL=${LLM_MODEL:-$ROOT/models/llm/qwen2.5-1.5b-instruct-q4_k_m.gguf}
WHISPER_MODEL=${WHISPER_MODEL:-$ROOT/models/whisper/ggml-base.en-q5_1.bin}

pkill -f whisper-server || true
pkill -f llama-server   || true

$ROOT/src/whisper.cpp/build/bin/whisper-server \
  -m "$WHISPER_MODEL" -l en -t 4 --host 127.0.0.1 --port 8080 \
  > /tmp/robot_voice_whisper.log 2>&1 &

#: on Jetson the GPU copy already lives in RAM, mmap would keep a 2nd copy in page cache
# -c 2048 / -np 1: small KV cache, one conversation at a time
$ROOT/src/llama.cpp/build/bin/llama-server \
  -m "$LLM_MODEL" -ngl 99 -c 2048 -np 1 -t 4 --jinja \
  --host 127.0.0.1 --port 8081 \
  > /tmp/robot_voice_llm.log 2>&1 &

echo "waiting for servers..."
for i in $(seq 1 60); do
  if curl -sf http://127.0.0.1:8081/health >/dev/null; then break; fi; sleep 1
done
curl -sf http://127.0.0.1:8081/health && echo " llama-server OK"
pgrep -f whisper-server >/dev/null && echo "whisper-server OK"
echo "check RAM with: tegrastats  (or: free -h)"
