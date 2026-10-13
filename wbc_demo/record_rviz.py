import argparse
import re
import subprocess
import time
from pathlib import Path

import numpy as np
from PyQt5.QtGui import QImage, QWindow
from PyQt5.QtWidgets import QApplication


def main():
    parser = argparse.ArgumentParser(description="Record only this demo's RViz window")
    parser.add_argument("--seconds", type=float, default=90.0)
    args = parser.parse_args()
    app = QApplication([])
    import cv2

    deadline = time.monotonic() + 60.0
    window = None
    while time.monotonic() < deadline:
        tree = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
        match = re.search(r'(0x[0-9a-f]+) "[^"\n]*fixed_base\.rviz[^"\n]*".*\("rviz2" "rviz2"\)', tree)
        if match:
            window = int(match.group(1), 16)
            break
        time.sleep(0.5)
    if window is None:
        raise RuntimeError("Demo RViz window not found")
    foreign_window = QWindow.fromWinId(window)
    foreign_window.raise_()
    foreign_window.requestActivate()
    app.processEvents()
    output = Path(__file__).parent / "logs"
    output.mkdir(exist_ok=True)
    writer = None
    frames = 0
    beginning = time.monotonic()
    try:
        while time.monotonic() - beginning < args.seconds:
            app.processEvents()
            shot = app.primaryScreen().grabWindow(window)
            if shot.isNull():
                raise RuntimeError("RViz window closed during recording")
            image = shot.toImage().convertToFormat(QImage.Format_RGB888)
            width, height = image.width(), image.height()
            pixels = image.bits()
            pixels.setsize(image.byteCount())
            frame = np.frombuffer(pixels, dtype=np.uint8).reshape(height, image.bytesPerLine())[:, : width * 3]
            frame = cv2.cvtColor(frame.reshape(height, width, 3), cv2.COLOR_RGB2BGR)
            if writer is None:
                writer = cv2.VideoWriter(
                    str(output / "rviz-demo.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (width, height)
                )
                if not writer.isOpened():
                    raise RuntimeError("Cannot create MP4 recording")
                shot.save(str(output / "rviz-start.png"))
            writer.write(frame)
            frames += 1
            if frames % 50 == 0:
                shot.save(str(output / "rviz-latest.png"))
            time.sleep(max(0.0, beginning + frames / 10.0 - time.monotonic()))
    finally:
        if writer is not None:
            writer.release()
    print(f"Recorded {frames} frames to {output / 'rviz-demo.mp4'}", flush=True)


if __name__ == "__main__":
    main()
