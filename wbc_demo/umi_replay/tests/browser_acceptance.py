import argparse
import asyncio
import base64
import json
import time
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np
import websockets


async def validate(args):
    args.output.mkdir(parents=True, exist_ok=True)
    result = {"trusted_pointer_actions": True, "executed_motion": args.execute}
    async with websockets.connect(args.websocket, max_size=12_000_000) as websocket:
        identifier = 0

        async def call(method, parameters):
            nonlocal identifier
            identifier += 1
            await websocket.send(json.dumps({"id": identifier, "method": method, "params": parameters}))
            while True:
                response = json.loads(await asyncio.wait_for(websocket.recv(), 30))
                if response.get("id") == identifier:
                    if response["type"] != "success":
                        raise RuntimeError(response)
                    return response["result"]

        session = await call("session.new", {"capabilities": {}})
        result["browser"] = session["capabilities"]["browserVersion"]
        context = (await call("browsingContext.getTree", {}))["contexts"][0]["context"]
        await call("browsingContext.setViewport", {"context": context, "viewport": {"width": 1400, "height": 1400}})
        await call("browsingContext.navigate", {"context": context, "url": args.url, "wait": "complete"})

        async def evaluate(expression):
            value = await call(
                "script.evaluate", {"target": {"context": context}, "expression": expression, "awaitPromise": True}
            )
            if value["type"] != "success":
                raise RuntimeError(value)
            return json.loads(value["result"]["value"])

        def status():
            with urlopen(args.url + "/api/status", timeout=5) as response:
                return json.load(response)

        async def wait_for(predicate, timeout=30):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                value = status()
                if predicate(value):
                    return value
                await asyncio.sleep(0.2)
            raise TimeoutError(status())

        async def center(selector):
            return await evaluate(
                "JSON.stringify((()=>{const element=document.querySelector("
                + json.dumps(selector)
                + ");element.scrollIntoView({block:'center'});const rect=element.getBoundingClientRect();return {x:Math.round(rect.x+rect.width/2),y:Math.round(rect.y+rect.height/2),disabled:!!element.disabled};})())"
            )

        async def pointer(actions):
            return await call(
                "input.performActions",
                {
                    "context": context,
                    "actions": [
                        {"type": "pointer", "id": "mouse", "parameters": {"pointerType": "mouse"}, "actions": actions}
                    ],
                },
            )

        async def click(selector):
            point = await center(selector)
            assert not point["disabled"], (selector, status())
            await pointer(
                [
                    {"type": "pointerMove", "x": point["x"], "y": point["y"]},
                    {"type": "pointerDown", "button": 0},
                    {"type": "pointerUp", "button": 0},
                ]
            )
            await asyncio.sleep(0.5)

        async def screenshot(name):
            await evaluate("JSON.stringify(window.scrollTo(0,0) ?? true)")
            value = await call("browsingContext.captureScreenshot", {"context": context, "origin": "document"})
            (args.output / name).write_bytes(base64.b64decode(value["data"]))

        def image():
            with urlopen(args.url + "/frame/ExternalA", timeout=5) as response:
                return cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)

        await wait_for(lambda state: state["ready"])
        await asyncio.sleep(1)
        loaded = await evaluate("JSON.stringify([...document.querySelectorAll('img')].map(image=>image.naturalWidth))")
        assert len(loaded) == 4 and min(loaded) > 0, loaded
        result["loaded_image_widths"] = loaded
        await screenshot("web-before-drag.png")
        before = status()
        pixels_before = image()
        point = await center('[data-orbit="0"]')
        await pointer(
            [
                {"type": "pointerMove", "x": point["x"], "y": point["y"]},
                {"type": "pointerDown", "button": 0},
                {"type": "pointerMove", "x": point["x"] + 100, "y": point["y"] + 25, "duration": 500},
                {"type": "pointerUp", "button": 0},
            ]
        )
        await wait_for(lambda state: state.get("command_count", 0) > before.get("command_count", 0))
        await asyncio.sleep(1)
        after = status()
        pixel_change = float(np.abs(image().astype(float) - pixels_before.astype(float)).mean())
        assert abs(after["camera_orbits"][0][0] - before["camera_orbits"][0][0]) > 0.4, (before, after)
        assert pixel_change > 2, pixel_change
        result["drag"] = {
            "before": before["camera_orbits"][0],
            "after": after["camera_orbits"][0],
            "mean_absolute_pixel_change": pixel_change,
        }
        await screenshot("web-after-drag.png")
        point = await center('[data-orbit="1"]')
        distance = after["camera_orbits"][1][2]
        await call(
            "input.performActions",
            {
                "context": context,
                "actions": [
                    {
                        "type": "wheel",
                        "id": "wheel",
                        "actions": [
                            {
                                "type": "scroll",
                                "x": point["x"],
                                "y": point["y"],
                                "deltaX": 0,
                                "deltaY": -180,
                                "duration": 200,
                            }
                        ],
                    }
                ],
            },
        )
        await wait_for(lambda state: abs(state["camera_orbits"][1][2] - distance) > 0.1)
        result["wheel"] = {"before_distance": distance, "after_distance": status()["camera_orbits"][1][2]}
        await click('[data-command="reset_views"]')
        await wait_for(lambda state: state.get("last_command") == "reset_views")
        np.testing.assert_allclose(status()["camera_orbits"], [[np.pi / 4, 0.35, 2.4], [-np.pi / 2, 0.25, 1.9]])
        result["reset_button"] = "passed"
        (args.output / "browser-validation.json").write_text(json.dumps(result, indent=2) + "\n")
        if args.execute:
            await click("#start")
            await wait_for(lambda state: state["stage"] == "replaying" and state["source_time_s"] > 1, timeout=180)
            await click('[data-command="pause"]')
            paused = await wait_for(lambda state: state["stage"] == "paused")
            await asyncio.sleep(1.5)
            delta = abs(status()["source_time_s"] - paused["source_time_s"])
            assert delta < 1e-8, delta
            result["pause_button_phase_delta_s"] = delta
            await screenshot("web-paused.png")
            await click('[data-command="resume"]')
            await wait_for(lambda state: state["stage"] == "replaying")
            result["resume_button"] = "passed"
            (args.output / "browser-validation.json").write_text(json.dumps(result, indent=2) + "\n")
            completed = await wait_for(lambda state: not state["busy"], timeout=1200)
            assert completed["stage"] == "finished", completed
            result["final_status"] = completed
        await call("input.releaseActions", {"context": context})
        await call("session.end", {})
    (args.output / "browser-validation.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result), flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="Real Firefox pointer/button acceptance; --execute explicitly enables robot replay"
    )
    parser.add_argument("--websocket", default="ws://127.0.0.1:9223/session")
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    asyncio.run(validate(parser.parse_args()))


if __name__ == "__main__":
    main()
