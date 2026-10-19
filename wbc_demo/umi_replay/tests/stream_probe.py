import argparse
import concurrent.futures
import json
import time
from pathlib import Path
from urllib.request import urlopen

import numpy as np


def stream_sample(url, view, duration):
    rows = []
    end = time.monotonic() + duration
    with urlopen(url + "/stream/" + view, timeout=5) as response:
        while time.monotonic() < end:
            line = response.readline()
            if not line:
                raise RuntimeError("MJPEG connection closed")
            if line.strip() != b"--frame":
                continue
            headers = {}
            while True:
                line = response.readline().strip()
                if not line:
                    break
                name, value = line.split(b":", 1)
                headers[name.lower()] = value.strip()
            size = int(headers[b"content-length"])
            if len(response.read(size)) != size:
                raise RuntimeError("Truncated MJPEG frame")
            rows.append(
                {
                    "monotonic_s": time.monotonic(),
                    "bytes": size,
                    "version": int(headers[b"x-frame-version"]) if b"x-frame-version" in headers else None,
                }
            )
    if len(rows) < 2:
        raise RuntimeError("Not enough MJPEG frames to measure delivery")
    span = rows[-1]["monotonic_s"] - rows[0]["monotonic_s"]
    gaps = np.diff([row["monotonic_s"] for row in rows]) * 1000
    return {
        "view": view,
        "delivered_fps": (len(rows) - 1) / span,
        "gap_ms_p50_p95_p99_max": np.percentile(gaps, [50, 95, 99, 100]).tolist(),
        "mean_jpeg_bytes": float(np.mean([row["bytes"] for row in rows])),
        "samples": rows,
    }


def status_sample(url, duration):
    rows = []
    end = time.monotonic() + duration
    while time.monotonic() < end:
        begin = time.monotonic()
        with urlopen(url + "/api/status", timeout=5) as response:
            state = json.load(response)
        rows.append({"monotonic_s": time.monotonic(), "rtt_s": time.monotonic() - begin, "state": state})
        time.sleep(0.1)
    return rows


def main():
    parser = argparse.ArgumentParser(
        description="Read-only local MJPEG delivery and ROS-receive probe; does not move robot"
    )
    parser.add_argument("--url", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=15)
    args = parser.parse_args()
    if args.seconds < 2 or not args.url.startswith("http://127.0.0.1:"):
        parser.error("Use a loopback URL and at least two seconds")
    with concurrent.futures.ThreadPoolExecutor(4) as pool:
        streams = [
            pool.submit(stream_sample, args.url, view, args.seconds) for view in ("UmiEgo", "ExternalA", "ExternalB")
        ]
        statuses = pool.submit(status_sample, args.url, args.seconds)
        result = {"streams": [future.result() for future in streams], "statuses": statuses.result()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps([{key: value for key, value in stream.items() if key != "samples"} for stream in result["streams"]])
    )


if __name__ == "__main__":
    main()
