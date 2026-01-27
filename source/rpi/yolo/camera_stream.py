"""MJPEG camera streaming server for RoboSort.

Flow:
- USB camera -> /dev/video0 (or index 0)
- Flask serves MJPEG at /video
- ngrok exposes port 5000 to the internet
- Android app loads the ngrok https URL and displays the stream

Run on Raspberry Pi:
  python3 camera_stream.py

Then in another terminal:
  ngrok http 5000

Open:
  https://<your-ngrok-domain>/  (test page)
  https://<your-ngrok-domain>/video  (raw MJPEG)
"""

from __future__ import annotations

import os
import time

import cv2
from flask import Flask, Response


def _open_camera() -> cv2.VideoCapture:
    # Prefer Linux device path when available.
    device = os.getenv("CAMERA_DEVICE", "/dev/video0")

    cap: cv2.VideoCapture
    if device.isdigit():
        cap = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

    # Optional tuning
    width = int(os.getenv("CAMERA_WIDTH", "640"))
    height = int(os.getenv("CAMERA_HEIGHT", "480"))
    fps = int(os.getenv("CAMERA_FPS", "30"))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    return cap


app = Flask(__name__)
_camera = _open_camera()


def gen_frames():
    """Generate MJPEG frames."""
    # If the camera fails temporarily, retry.
    while True:
        if not _camera.isOpened():
            time.sleep(0.2)
            continue

        success, frame = _camera.read()
        if not success or frame is None:
            time.sleep(0.01)
            continue

        # Encode as JPEG
        ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            continue

        jpg = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: "
            + str(len(jpg)).encode("ascii")
            + b"\r\n\r\n"
            + jpg
            + b"\r\n"
        )


@app.get("/")
def index():
    # Simple test page (useful in browser)
    return (
        "<html><head><title>RoboSort Camera</title></head>"
        "<body style='margin:0;background:#111;display:flex;align-items:center;justify-content:center;height:100vh'>"
        "<img src='/video' style='max-width:100%;max-height:100%' />"
        "</body></html>"
    )


@app.get("/video")
def video():
    return Response(gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", "5000"))

    # threaded=True allows multiple clients (Android + browser) to connect.
    app.run(host=host, port=port, threaded=True)
