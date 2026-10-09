"""The simulation launcher (sim/launcher): its tiles and thumbnails, and a
world's life from a tile to Exit, with a stand-in for the driver station (no
Gazebo, so it runs in a second)."""
import asyncio
import io
import os
import sys
import tempfile
import textwrap
import unittest

from aiohttp.test_utils import AioHTTPTestCase
from PIL import Image

import worldfiles  # noqa: F401  (puts sim/ on the path)
from launcher import __main__ as launcher  # noqa: E402

FAKE_STATION = textwrap.dedent("""\
    import signal, sys, time
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    print("\\x1b[1;33mWarning\\x1b[0m starting", flush=True)
    print("driver station on http://127.0.0.1:9/ (Ctrl-C to stop)", flush=True)
    while True:
        time.sleep(0.05)
""")


class Tiles(unittest.TestCase):
    def test_missions_first_in_rule_order(self):
        tiles = launcher.worlds()
        ids = [tile["id"] for tile in tiles]
        self.assertEqual(ids[:4], ["urc_astrobiology", "urc_delivery", "urc_equipment_servicing", "urc_autonomy"])
        self.assertFalse([i for i in ids if i.startswith("tmp")], "a test's temporary world is not a tile")
        for tile in tiles:
            self.assertTrue((launcher.WORLDS / f"{tile['id']}.sdf").is_file())
        self.assertEqual(tiles[0]["subtitle"], "URC 2027 1.b · 20–30 min")

    def test_thumbnails_are_small_squares(self):
        tiles = [tile for tile in launcher.worlds() if tile["thumb"]]
        if not tiles:
            self.skipTest("no orthophoto maps (pixi run sim-maps)")
        for tile in tiles:
            with self.subTest(tile["id"]):
                body = launcher.thumbnail(tile["id"], tile["thumb"])
                self.assertLess(len(body), 200_000)
                self.assertEqual(Image.open(io.BytesIO(body)).size, (launcher.THUMB_PX, launcher.THUMB_PX))


class Life(AioHTTPTestCase):
    async def asyncSetUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        fake = os.path.join(self.tmp.name, "station.py")
        with open(fake, "w") as f:
            f.write(FAKE_STATION)
        self.saved = launcher.STATION, launcher.GRACE
        launcher.STATION, launcher.GRACE = fake, 0.2
        self.launcher = launcher.Launcher()
        await super().asyncSetUp()

    async def asyncTearDown(self):
        await self.launcher.stop()
        await super().asyncTearDown()
        launcher.STATION, launcher.GRACE = self.saved
        self.tmp.cleanup()

    async def get_application(self):
        return launcher.make_app(self.launcher)

    async def until(self, status, timeout=10.0):
        for _ in range(int(timeout / 0.05)):
            if self.launcher.status == status:
                return
            await asyncio.sleep(0.05)
        self.fail(f"status {self.launcher.status}, not {status}")

    async def test_launch_shows_the_station_and_exit_stops_it(self):
        async with self.client.ws_connect("/ws") as ws:
            self.assertEqual((await ws.receive_json())["status"], "idle")
            response = await self.client.post("/api/launch", json={"world": "rover_test"})
            self.assertEqual(response.status, 202)
            await self.until("running")
            process = self.launcher.process
            state = await ws.receive_json()
            while state["status"] != "running":
                state = await ws.receive_json()
            self.assertEqual((state["world"], state["url"]), ("rover_test", "http://127.0.0.1:9/"))
            self.assertIn("Warning starting", state["log"], "terminal colours are stripped")
            await self.client.post("/api/stop")
            await self.until("idle")
            self.assertIsNotNone(process.returncode, "Exit stops the station")
            self.assertIsNone(self.launcher.world)

    async def test_closing_the_page_stops_the_simulation(self):
        async with self.client.ws_connect("/ws") as ws:
            await ws.receive_json()
            await self.client.post("/api/launch", json={"world": "rover_test"})
            await self.until("running")
        await self.until("idle", timeout=launcher.GRACE + 5.0)

    async def test_an_unknown_world_is_refused(self):
        response = await self.client.post("/api/launch", json={"world": "../etc/passwd"})
        self.assertEqual(response.status, 404)
        self.assertIsNone(self.launcher.process)
        self.assertEqual((await self.client.get("/thumb/rover_test")).status, 404)


if __name__ == "__main__":
    unittest.main()
