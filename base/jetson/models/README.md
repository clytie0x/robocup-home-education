# Models for the Jetson follower

The model files are not in Git. On the Jetson they live in `~/hmc-follow/models/`. Downloaded 2026-11-13 over HTTPS
with normal certificate checks.

## Downloaded files

| File | Source URL | Bytes | Licence | SHA-256 |
|---|---|---|---|---|
| yolo11n.pt | https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt | 5613764 | AGPL-3.0 (Ultralytics) | 0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1 |
| yolov8n.pt | https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt | 6549796 | AGPL-3.0 (Ultralytics) | f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36 |
| yolo11n-pose.pt (2026-11-15, owner approved, for pointing) | https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt | 6255593 | AGPL-3.0 (Ultralytics) | 869e83fcdffdc7371fa4e34cd8e51c838cc729571d1635e5141e3075e9319dc0 |
| yolox_nano.pth | https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_nano.pth | 7694953 | Apache-2.0 (Megvii) | cd28f55fbbc1829f99d9ac9b38a16d259a22889739c8728ea877610201feff7b |
| yolox_tiny.pth | https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_tiny.pth | 40755013 | Apache-2.0 (Megvii) | 9de513de589ac98bb92d3bca53b5af7b9acfa9b0bacb831f7999d0f7afaee8f0 |
| osnet_x0_25_msmt17_combineall_256x128_amsgrad_ep150_stp60_lr0.0015_b64_fb10_softmax_labelsmooth_flip_jitter.pth | https://huggingface.co/kaiyangzhou/osnet/resolve/a5c5cc037c24235cda3b21085b93ad77c9616224/osnet_x0_25_msmt17_combineall_256x128_amsgrad_ep150_stp60_lr0.0015_b64_fb10_softmax_labelsmooth_flip_jitter.pth | 9336983 | MIT (torchreid, Kaiyang Zhou) | cf55163d78fc44c62c82f85ab62d39f10438679b5abe8c698ae08cfa84aa6e18 |
| two_people_forest.jpg (bench image) | https://upload.wikimedia.org/wikipedia/commons/8/86/Two_females_in_forest_walking_trough_high_grass.jpg (Wikimedia Commons, Rachel E. Molenda, U.S. Fish and Wildlife Service) | 1006261 | Public domain (US federal work) | 5687b3b41e4bb0ac263c71ef3dbb5e049ed6f34d10e97a12cbbd5dc5d7c54b5f |

Checks: the OSNet SHA-256 equals the Hugging Face LFS object id; the image SHA-1 equals the one Commons lists
(0a93dadac17c8569e3aa1d5af4ce723070876b17). GitHub lists no digest for the four release assets; their byte sizes
match the release listing.

Model code used only for the export (not shipped): YOLOX source at tag 0.1.1rc0 (commit
e1052df71842031413f6030723c3607b839c80ce, https://github.com/Megvii-BaseDetection/YOLOX, Apache-2.0) and
`torchreid/models/osnet.py` at commit f8cd150fdf77e8d9e1ed143b7f308c2c609ded50 of
https://github.com/KaiyangZhou/deep-person-reid (MIT).

## Pose model export, 2026-11-15

Laptop scratch venv on Python 3.13 with the system torch 2.13.0+cpu, `ultralytics 8.4.213`, `onnx 1.23.2`,
`onnxslim` (venv deleted after the export): `YOLO("yolo11n-pose.pt").export(format="onnx", imgsz=(256, 320),
opset=17, simplify=True, dynamic=False)` gives `yolo11n-pose_256x320.onnx` (11638357 bytes, SHA-256
97d290f0e0b5180866614b0f2804d73dbbf6834576b89fb188e4ac677dfa070b), input `images` 1x3x256x320, output `output0`
1x56x1680 (4 box, 1 score, 17 COCO keypoints x (x, y, visibility)). Engine on the Jetson:
`/usr/bin/trtexec --onnx=yolo11n-pose_256x320.onnx --saveEngine=yolo11n-pose_256x320.engine --fp16 --skipInference`.

## Export to ONNX (laptop, scratch venv, Python 3.12)

Venv: `torch 2.15.0+cpu` (from https://download.pytorch.org/whl/cpu), `ultralytics 8.4.211`, `onnx 1.23.2`, `onnxslim`,
`onnxruntime`, `loguru`. Opset 17, input 256 x 320 (height x width) for the detectors: a 320 x 240 frame fits with
16 rows of padding at the bottom.

```python
# Ultralytics (yolo11n, yolov8n)
from ultralytics import YOLO
YOLO("yolo11n.pt").export(format="onnx", imgsz=(256, 320), opset=17, simplify=True, dynamic=False)

# YOLOX (nano: width 0.25, depthwise; tiny: width 0.375), with the YOLOX 0.1.1rc0 source on sys.path
from yolox.models import YOLOX, YOLOPAFPN, YOLOXHead
ch = [256, 512, 1024]
model = YOLOX(YOLOPAFPN(0.33, 0.25, in_channels=ch, depthwise=True), YOLOXHead(80, 0.25, in_channels=ch, depthwise=True))
for m in model.modules():
    if isinstance(m, torch.nn.BatchNorm2d):
        m.eps, m.momentum = 1e-3, 0.03
model.load_state_dict(torch.load("yolox_nano.pth", map_location="cpu", weights_only=False)["model"])
model.eval(); model.head.decode_in_inference = True
torch.onnx.export(model, torch.zeros(1, 3, 256, 320), "yolox_nano_256x320.onnx", input_names=["images"],
                  output_names=["output"], opset_version=17, dynamo=False)

# YOLOX-tiny: the same steps with width 0.375 and depthwise=False
model = YOLOX(YOLOPAFPN(0.33, 0.375, in_channels=ch, depthwise=False), YOLOXHead(80, 0.375, in_channels=ch, depthwise=False))
for m in model.modules():
    if isinstance(m, torch.nn.BatchNorm2d):
        m.eps, m.momentum = 1e-3, 0.03
model.load_state_dict(torch.load("yolox_tiny.pth", map_location="cpu", weights_only=False)["model"])
model.eval(); model.head.decode_in_inference = True
torch.onnx.export(model, torch.zeros(1, 3, 256, 320), "yolox_tiny_256x320.onnx", input_names=["images"],
                  output_names=["output"], opset_version=17, dynamo=False)

# OSNet x0_25 (torchreid osnet.py), classifier weights dropped, dynamic batch
model = osnet.osnet_x0_25(num_classes=1, pretrained=False)
sd = {k: v for k, v in torch.load(OSNET_PTH, map_location="cpu", weights_only=False).items() if not k.startswith("classifier")}
model.load_state_dict(sd, strict=False); model.eval()
torch.onnx.export(model, torch.zeros(1, 3, 256, 128), "osnet_x0_25_msmt17_256x128.onnx", input_names=["input"],
                  output_names=["output"], opset_version=17, dynamo=False,
                  dynamic_axes={"input": {0: "batch"}, "output": {0: "batch"}})
```

Preprocessing the engines expect (see `detector.py`, `reid.py`): Ultralytics RGB / 255; YOLOX 0.1.1rc0 weights BGR 0-255
unscaled (RGB with mean and std, as that tag's demo does, found nobody on the bench image); OSNet RGB / 255 with
ImageNet mean and std, 256 x 128.

| ONNX | Bytes | SHA-256 |
|---|---|---|
| yolo11n_256x320.onnx | 10606884 | 6d12645bf4fa2d50e4e314aa15faf6687439088b077cfcedfa9399f8ced29a7b |
| yolov8n_256x320.onnx | 12716618 | ba1d60198823dd552fed0632b8b7613216599ae0680d0dd6991dffa078d863e9 |
| yolox_nano_256x320.onnx | 3770319 | eb29864fdb5c7fec4bc8ff548e63de4dedc43490ca7029620dc4a1b6beffe4cf |
| yolox_tiny_256x320.onnx | 20315641 | 32468756b2e35f0282e3de6c31bded50489854fd8a176c2e235080d02a818dde |
| osnet_x0_25_msmt17_256x128.onnx | 891539 | 7bd0332f88ce99ad9e05c20419a97eb82a98157510356688b6888d6296dcc6cb |

## TensorRT engines (Jetson, TensorRT 10.16.2.10, FP16)

```sh
cd ~/hmc-follow/models
for m in yolo11n_256x320 yolov8n_256x320 yolox_nano_256x320 yolox_tiny_256x320; do
  /usr/bin/trtexec --onnx=$m.onnx --saveEngine=$m.engine --fp16 --skipInference
done
/usr/bin/trtexec --onnx=osnet_x0_25_msmt17_256x128.onnx --saveEngine=osnet_x0_25_msmt17_256x128.engine --fp16 \
  --skipInference --minShapes=input:1x3x256x128 --optShapes=input:2x3x256x128 --maxShapes=input:8x3x256x128
```

Engines are tied to this Jetson's TensorRT version; rebuild them after a TensorRT upgrade.

## Bench memory figures

`bench.py` reports `gpu_engines_buffers_mib`: engines plus device buffers, excluding the CUDA context (the first
`cudaMemGetInfo` call creates the context before the baseline is taken). On the Orin the GPU shares RAM with the CPU.

## Bag segmenter tools (front camera, laptop)

The bag segmentation models (`bag11n-seg-v3_480x640.onnx` and later, classes bag and handle) are not in Git either:
they are on the owner's laptop as split zips (join the parts into one zip to read them; the parts stay as they are).

- `tools/capture_front.py OUT_DIR [--host 100.108.202.48|192.168.31.137]` collects new frames for training: it GETs
  the Jetson's `:8082/snap.jpg` (nothing else) and keeps a frame only when the view changed (mean abs difference of
  a grey thumbnail), as `OUT_DIR/front_YYYYMMDD-HHMMSS.jpg`.
- `tools/eval_bag_seg.py FRAMES_DIR --out SHEETS_DIR --model A.onnx --model B.zip:member.onnx [--labels LABELS_DIR]`
  runs the models on onnxruntime CPU (frame resized to 640 x 480, RGB / 255), prints each frame's bag and handle
  scores and, with YOLO-seg labels, each model's bag recall and mean mask IoU at IoU 0.5 and its false bags on frames
  with no bag; it writes contact sheets.
