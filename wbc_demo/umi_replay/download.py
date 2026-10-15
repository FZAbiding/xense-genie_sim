import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

from huggingface_hub import HfApi, snapshot_download

DATASET = "TacVerse/xtac-umi-g1-block-to-box"
REVISION = "8ad651578fda4724514387e2d3d3551c3c9631f9"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def download(destination, revision=REVISION, max_bytes=400_000_000, metadata_only=False):
    destination = Path(destination)
    info = HfApi().dataset_info(DATASET, revision=revision, files_metadata=True)
    files = [
        item
        for item in info.siblings
        if not metadata_only or item.rfilename == "README.md" or item.rfilename.startswith(("meta/", "data/"))
    ]
    if any(item.size is None for item in files):
        raise RuntimeError("Hub returned unknown file sizes; refusing unbounded download")
    total = sum(item.size for item in files)
    if total > max_bytes:
        raise RuntimeError(f"Download needs {total} bytes, exceeding the explicit limit {max_bytes}")
    print(f"Downloading {len(files)} files / {total} bytes at immutable revision {info.sha}", flush=True)
    snapshot_download(
        DATASET,
        repo_type="dataset",
        revision=info.sha,
        local_dir=destination,
        allow_patterns=[item.rfilename for item in files],
        max_workers=3,
    )
    records = []
    for item in files:
        path = destination / item.rfilename
        if path.stat().st_size != item.size:
            raise RuntimeError(f"Size mismatch: {item.rfilename}")
        records.append({"path": item.rfilename, "bytes": item.size, "sha256": sha256(path)})
    manifest = {
        "dataset": DATASET,
        "revision": info.sha,
        "downloaded_at": datetime.now(timezone.utc).isoformat(),
        "license": "cc-by-sa-4.0",
        "metadata_only": metadata_only,
        "total_bytes": total,
        "files": records,
    }
    (destination / "download-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest
