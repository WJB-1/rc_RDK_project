#!/usr/bin/env python3
"""Print the Horizon BPU tensor contract for the YOLOv8 road-segmentation model."""

import argparse
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MODEL = PROJECT_ROOT / "models" / "yolov8n_seg_road_x5.bin"
DEFAULT_IMAGE_DIR = PROJECT_ROOT / "perception" / "test_img"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--image", type=Path, help="One BGR test image")
    parser.add_argument("--input-width", type=int, default=640)
    parser.add_argument("--input-height", type=int, default=640)
    return parser.parse_args()


def bgr_to_nv12(image):
    height, width = image.shape[:2]
    area = width * height
    yuv_i420 = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    y = yuv_i420[:area]
    uv = yuv_i420[area:].reshape(2, area // 4).T.reshape(-1)
    return np.concatenate((y, uv))


def describe_tensor(prefix, tensor):
    properties = getattr(tensor, "properties", tensor)
    fields = {}
    for name in ("name", "shape", "dtype", "tensor_type", "layout", "scale_data", "shift", "quanti_type"):
        value = getattr(properties, name, None)
        if value is not None:
            fields[name] = str(value)
    print(f"{prefix}: {fields}")


def main():
    args = parse_args()
    if args.image is None:
        images = sorted(
            path for path in DEFAULT_IMAGE_DIR.iterdir()
            if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}
        )
        if not images:
            raise FileNotFoundError(f"no test image found in: {DEFAULT_IMAGE_DIR}")
        args.image = images[0]
        print(f"image not specified; using: {args.image}")
    if not args.model.is_file():
        raise FileNotFoundError(f"model not found: {args.model}")
    image = cv2.imread(str(args.image))
    if image is None:
        raise RuntimeError(f"unable to read image: {args.image}")

    try:
        from hobot_dnn import pyeasy_dnn as dnn
    except ImportError as error:
        raise RuntimeError("Run this script in the RDK environment with hobot_dnn installed") from error

    models = dnn.load(str(args.model))
    if not models:
        raise RuntimeError("dnn.load returned no models")
    model = models[0]
    print(f"model={args.model}")
    for index, tensor in enumerate(getattr(model, "inputs", [])):
        describe_tensor(f"input[{index}]", tensor)
    for index, tensor in enumerate(getattr(model, "outputs", [])):
        describe_tensor(f"output[{index}]", tensor)

    resized = cv2.resize(image, (args.input_width, args.input_height), interpolation=cv2.INTER_AREA)
    outputs = model.forward([bgr_to_nv12(resized)])
    print(f"forward_outputs={len(outputs)}")
    for index, output in enumerate(outputs):
        buffer = np.asarray(output.buffer)
        print(
            f"buffer[{index}]: shape={buffer.shape} dtype={buffer.dtype} "
            f"min={buffer.min():.6g} max={buffer.max():.6g}"
        )
        describe_tensor(f"buffer_properties[{index}]", output)


if __name__ == "__main__":
    main()
