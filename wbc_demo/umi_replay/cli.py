import argparse
import json
from pathlib import Path

from .conversion import convert
from .download import REVISION, download

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data/xtac-umi-g1-block-to-box"


def main():
    parser = argparse.ArgumentParser(description="Auditable XTac UMI download, conversion and fixed-base G2 replay")
    commands = parser.add_subparsers(dest="command", required=True)
    fetch = commands.add_parser("download")
    fetch.add_argument("--root", type=Path, default=DATA)
    fetch.add_argument("--revision", default=REVISION)
    fetch.add_argument("--metadata-only", action="store_true")
    convert_parser = commands.add_parser("convert")
    convert_parser.add_argument("--root", type=Path, default=DATA)
    convert_parser.add_argument("--episode", type=int, default=0)
    convert_parser.add_argument("--feature", choices=["observation.state", "action"], default="observation.state")
    convert_parser.add_argument("--max-interval", type=float, default=0.2)
    convert_parser.add_argument("--output", type=Path, required=True)
    preview_parser = commands.add_parser("preview")
    preview_parser.add_argument("--root", type=Path, default=DATA)
    preview_parser.add_argument("--input", type=Path, required=True)
    preview_parser.add_argument("--output", type=Path, required=True)
    analyze_parser = commands.add_parser("analyze")
    analyze_parser.add_argument("--input", type=Path, required=True)
    analyze_parser.add_argument("--run-directory", type=Path, required=True)
    evidence_parser = commands.add_parser("evidence")
    evidence_parser.add_argument("--run-directory", type=Path, required=True)
    evidence_parser.add_argument("--robot-video", type=Path, required=True)
    evidence_parser.add_argument("--source-video", type=Path, required=True)
    evidence_parser.add_argument("--eef-diagram", type=Path, required=True)
    evidence_parser.add_argument("--output", type=Path, required=True)
    evidence_parser.add_argument("--doc-media", type=Path)
    evidence_parser.add_argument("--synchronized-directory", type=Path)
    evidence_parser.add_argument("--review-directory", type=Path)
    synchronized_parser = commands.add_parser("sync-video")
    synchronized_parser.add_argument("--root", type=Path, default=DATA)
    synchronized_parser.add_argument("--input", type=Path, required=True)
    synchronized_parser.add_argument("--run-directory", type=Path, required=True)
    synchronized_parser.add_argument("--robot-video", type=Path, required=True)
    synchronized_parser.add_argument("--camera-trace", type=Path, required=True)
    synchronized_parser.add_argument("--max-camera-gap", type=float, default=0.4)
    synchronized_parser.add_argument("--max-source-phase-gap", type=float, default=0.1)
    synchronized_parser.add_argument("--output", type=Path, required=True)
    synchronized_parser.add_argument("--layout", choices=("dual", "quad"), default="dual")
    views_parser = commands.add_parser("view-scene")
    views_parser.add_argument("--root", type=Path, default=DATA)
    views_parser.add_argument("--input", type=Path, required=True)
    views_parser.add_argument("--config", type=Path, default=ROOT / "umi_replay/configs/g2_shared_world.json")
    views_parser.add_argument("--base-scene", type=Path, default=ROOT / "scene_stress.yaml")
    views_parser.add_argument("--output", type=Path, required=True)
    live_parser = commands.add_parser("live")
    live_parser.add_argument("--root", type=Path, default=DATA)
    live_parser.add_argument("--input", type=Path, required=True)
    live_parser.add_argument("--scene", type=Path, required=True)
    live_parser.add_argument("--output", type=Path, required=True)
    live_parser.add_argument("--config", type=Path, default=ROOT / "umi_replay/configs/g2_shared_world.json")
    live_parser.add_argument("--autoplay", action="store_true")
    live_parser.add_argument("--auto-close", action="store_true")
    live_parser.add_argument("--posture-policy", choices=("soft", "upright"), default="soft")
    live_parser.add_argument("--port", type=int, default=8765)
    live_parser.add_argument("--playback-mode", choices=("realtime", "validated"), default="realtime")
    live_parser.add_argument("--limits-profile", type=Path, default=ROOT / "umi_replay/configs/g2_realtime_limits.json")
    replay = commands.add_parser("replay")
    replay.add_argument("--input", type=Path, required=True)
    replay.add_argument("--config", type=Path, default=ROOT / "umi_replay/configs/g2_shared_world.json")
    replay.add_argument("--execute", action="store_true")
    replay.add_argument("--output", type=Path, required=True)
    replay.add_argument("--speed", type=float, default=0.5)
    replay.add_argument("--posture-policy", choices=("config", "soft", "upright"), default="config")
    replay.add_argument("--interactive-controls", action="store_true")
    replay.add_argument("--playback-mode", choices=("validated", "realtime"), default="validated")
    replay.add_argument("--limits-profile", type=Path, default=ROOT / "umi_replay/configs/g2_realtime_limits.json")
    args = parser.parse_args()
    if args.command == "download":
        download(args.root, args.revision, metadata_only=args.metadata_only)
    elif args.command == "convert":
        convert(args.root, args.episode, args.output, args.feature, args.max_interval)
    elif args.command == "preview":
        from .preview import preview

        preview(args.root, json.loads(args.input.read_text()), args.output)
    elif args.command == "analyze":
        from .analysis import analyze

        analyze(args.run_directory, json.loads(args.input.read_text()))
    elif args.command == "evidence":
        from .evidence import build_evidence

        build_evidence(args)
    elif args.command == "sync-video":
        from .synchronized_video import synchronized_video

        synchronized_video(args, json.loads(args.input.read_text()))
    elif args.command == "view-scene":
        from .camera_views import make_scene

        print(
            make_scene(
                args.base_scene,
                args.root,
                json.loads(args.input.read_text()),
                json.loads(args.config.read_text()),
                args.output,
            )
        )
    elif args.command == "live":
        from .live_view import live

        live(args, json.loads(args.input.read_text()))
    else:
        from .execution import run

        run(args, json.loads(args.input.read_text()), json.loads(args.config.read_text()))
