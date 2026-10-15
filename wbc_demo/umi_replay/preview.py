import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as parquet


def preview(root, converted, output):
    import cv2
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as pyplot

    root = Path(root)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    frames = converted["source_frames"]
    selected = converted["keyframe_indexes"]
    figure = pyplot.figure(figsize=(13, 5))
    spatial = figure.add_subplot(121, projection="3d")
    temporal = figure.add_subplot(122)
    for side, color in zip(range(2), ("tab:blue", "tab:orange")):
        positions = np.array([frame["eef"][side]["position"] for frame in frames])
        label = ("left", "right")[side]
        spatial.plot(*positions.T, color=color, label=label)
        spatial.scatter(*positions[selected].T, color=color, s=3)
        temporal.plot([frame["timestamp"] for frame in frames], positions[:, 2], color=color, label=label + " z")
    spatial.set(
        xlabel="world x (m)", ylabel="world y (m)", zlabel="world z (m)", title="Original EEF trajectories, 1:1 scale"
    )
    temporal.set(
        xlabel="source time (s)",
        ylabel="height relative to PICO world origin (m)",
        title="No robot/world calibration applied",
    )
    spatial.legend()
    temporal.legend()
    figure.tight_layout()
    figure.savefig(output / "source-trajectory.png", dpi=160)
    pyplot.close(figure)
    metadata = []
    for path in sorted((root / "meta/episodes").rglob("*.parquet")):
        metadata.extend(parquet.read_table(path).to_pylist())
    episode = next(item for item in metadata if item["episode_index"] == converted["episode_index"])
    info = json.loads((root / "meta/info.json").read_text())
    keys = ("observation.images.head_left", "observation.images.left_wrist", "observation.images.right_wrist")
    captures = []
    writer = None
    try:
        for key in keys:
            path = root / info["video_path"].format(
                video_key=key,
                chunk_index=episode[f"videos/{key}/chunk_index"],
                file_index=episode[f"videos/{key}/file_index"],
            )
            capture = cv2.VideoCapture(str(path))
            captures.append(capture)
            if not capture.isOpened() or abs(capture.get(cv2.CAP_PROP_FPS) - info["fps"]) > 0.01:
                raise RuntimeError(f"Missing video or unexpected video FPS: {path}")
            capture.set(cv2.CAP_PROP_POS_FRAMES, round(episode[f"videos/{key}/from_timestamp"] * info["fps"]))
        writer = cv2.VideoWriter(
            str(output / "source-episode.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), info["fps"], (960, 264)
        )
        if not writer.isOpened():
            raise RuntimeError("Cannot create source preview video")
        for frame in frames:
            panels = []
            for key, capture in zip(keys, captures):
                success, image = capture.read()
                if not success:
                    raise RuntimeError(f"Truncated source video at frame {frame['frame_index']}: {key}")
                panel = np.zeros((264, 320, 3), dtype=np.uint8)
                panel[24:] = cv2.resize(image, (320, 240))
                cv2.putText(
                    panel,
                    key.removeprefix("observation.images."),
                    (8, 17),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1,
                )
                panels.append(panel)
            composite = np.hstack(panels)
            label = f"dataset episode {converted['episode_index']} / frame {frame['frame_index']} / t={frame['timestamp']:.3f}s"
            cv2.putText(composite, label, (8, 258), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            writer.write(composite)
            if frame["frame_index"] in (0, len(frames) // 2, len(frames) - 1):
                cv2.imwrite(str(output / f"source-frame-{frame['frame_index']:05d}.png"), composite)
    finally:
        for capture in captures:
            capture.release()
        if writer is not None:
            writer.release()
    print(f"Source preview: {output}", flush=True)
