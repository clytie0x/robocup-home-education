#!/usr/bin/env bash
# Start / stop / check the STT (whisper) and LLM (llama) servers.
#   servers.sh start        whisper + llama   (for brain:=llm)
#   servers.sh start-stt    whisper only      (for brain:=rules, saves ~1.4 GB RAM)
#   servers.sh stop         stop both
#   servers.sh restart      stop, then start both
#   servers.sh status       which servers are up + RAM
#   servers.sh get-small    download the more accurate whisper small.en model (once, ~190 MB)
# Whisper model: small.en is used automatically once downloaded (better with accents), else base.en.
# Force one with: WHISPER_MODEL=/opt/robot_voice/models/whisper/ggml-base.en-q5_1.bin servers.sh start-stt
ROOT=/opt/robot_voice
LLM_MODEL=${LLM_MODEL:-$ROOT/models/llm/qwen2.5-1.5b-instruct-q4_k_m.gguf}
SMALL=$ROOT/models/whisper/ggml-small.en-q5_1.bin
BASE=$ROOT/models/whisper/ggml-base.en-q5_1.bin
if [ -z "$WHISPER_MODEL" ]; then
  if [ -f "$SMALL" ]; then WHISPER_MODEL=$SMALL; else WHISPER_MODEL=$BASE; fi
fi

start_whisper() {
  if pgrep -f whisper-server >/dev/null; then echo "whisper-server already running"; return; fi
  echo "whisper model: $(basename "$WHISPER_MODEL")"
  $ROOT/src/whisper.cpp/build/bin/whisper-server \
    -m "$WHISPER_MODEL" -l en -t 4 --host 127.0.0.1 --port 8080 \
    > /tmp/robot_voice_whisper.log 2>&1 &
  for i in $(seq 1 20); do
    if curl -s -o /dev/null http://127.0.0.1:8080/ 2>/dev/null; then echo "whisper-server OK"; return; fi
    sleep 0.5
  done
  echo "whisper-server FAILED -> tail /tmp/robot_voice_whisper.log"
}

start_llama() {
  if pgrep -f llama-server >/dev/null; then echo "llama-server already running"; return; fi
  $ROOT/src/llama.cpp/build/bin/llama-server \
    -m "$LLM_MODEL" -ngl 99 -c 2048 -np 1 -t 4 --jinja \
    --host 127.0.0.1 --port 8081 \
    > /tmp/robot_voice_llm.log 2>&1 &
  echo -n "waiting for llama-server"
  for i in $(seq 1 60); do
    if curl -sf http://127.0.0.1:8081/health >/dev/null; then echo " OK"; return; fi
    if ! pgrep -f llama-server >/dev/null; then break; fi
    echo -n "."; sleep 1
  done
  echo " FAILED -> tail /tmp/robot_voice_llm.log"
}

stop_all() {
  pkill -f llama-server   && echo "llama-server stopped"   || echo "llama-server was not running"
  pkill -f whisper-server && echo "whisper-server stopped" || echo "whisper-server was not running"
}

status() {
  if pgrep -f whisper-server >/dev/null; then
    echo "whisper-server : running ($(pgrep -af whisper-server | grep -o 'ggml-[^ ]*' | head -1))"
  else echo "whisper-server : stopped"; fi
  if curl -sf http://127.0.0.1:8081/health >/dev/null; then echo "llama-server   : running (ready)"
  elif pgrep -f llama-server >/dev/null; then echo "llama-server   : starting..."
  else echo "llama-server   : stopped"; fi
  free -h | head -2
}

get_small() {
  if [ -f "$SMALL" ]; then echo "already downloaded: $SMALL"; return; fi
  bash $ROOT/src/whisper.cpp/models/download-ggml-model.sh small.en-q5_1 $ROOT/models/whisper \
    && echo "done. restart with: $0 stop && $0 start-stt"
}

case "${1:-status}" in
  start)      start_whisper; start_llama ;;
  start-stt)  start_whisper ;;
  stop)       stop_all ;;
  restart)    stop_all; sleep 1; start_whisper; start_llama ;;
  status)     status ;;
  get-small)  get_small ;;
  *) echo "usage: $0 {start|start-stt|stop|restart|status|get-small}"; exit 1 ;;
esac
