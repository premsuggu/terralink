"""Studio server over a real websocket (tornado test client; no ROS, no simulator)."""
import json

import numpy as np
from tornado.testing import AsyncHTTPTestCase, gen_test
from tornado.websocket import websocket_connect

from nav.studio import protocol
from nav.studio.commands import CommandError
from nav.studio.server import StudioServer


class _Base(AsyncHTTPTestCase):
    def get_app(self):
        self.received = []
        self.reject = False

        def on_command(cmd):
            if self.reject:
                raise CommandError("goal is outside the map")
            self.received.append(cmd)

        self.server = StudioServer(on_command, hello={"site": "test"}, ioloop=self.io_loop)
        return self.server.app

    async def connect(self):
        ws = await websocket_connect(f"ws://127.0.0.1:{self.get_http_port()}/ws")
        hello = json.loads(await ws.read_message())
        return ws, hello

    @staticmethod
    def sample_map(version=1):
        elev = np.linspace(0, 1, 30 * 30, dtype=np.float32).reshape(30, 30)
        valid = np.zeros((30, 30), dtype=bool)
        valid[5:20, 8:25] = True
        return protocol.pack_map(elev, valid, valid, 0.1, 0.0, 0.0, version)


class TestHandshake(_Base):
    @gen_test
    async def test_a_new_client_gets_hello_with_the_protocol_version(self):
        ws, hello = await self.connect()
        assert hello["t"] == "hello" and hello["protocol"] == protocol.PROTOCOL_VERSION and hello["site"] == "test"
        ws.close()

    @gen_test
    async def test_a_late_client_receives_the_latest_state_and_map_at_once(self):
        self.server.publish_state({"x": 1})
        self.server.publish_map(self.sample_map(version=5))
        await self.io_loop.run_in_executor(None, lambda: None)          # let the scheduled broadcasts run
        ws, _ = await self.connect()
        state = json.loads(await ws.read_message())
        frame = await ws.read_message()
        assert state == {"t": "state", "x": 1}
        assert protocol.unpack_map(frame).header["v"] == 5
        ws.close()


class TestCommands(_Base):
    @gen_test
    async def test_a_valid_command_reaches_the_handler_validated(self):
        ws, _ = await self.connect()
        await ws.write_message(json.dumps({"t": "uav_vel", "vx": 9, "scale": 0.5}))        # 9 is clamped to 1
        await ws.write_message(json.dumps({"t": "ping"}))
        assert json.loads(await ws.read_message()) == {"t": "pong"}                          # ping is answered by the server itself
        assert len(self.received) == 1
        assert self.received[0].kind == "uav_vel" and self.received[0].data["vx"] == 1.0
        ws.close()

    @gen_test
    async def test_garbage_gets_an_error_frame_and_never_reaches_the_handler(self):
        ws, _ = await self.connect()
        for bad in ("not json", '{"t":"uav_vel","vx":NaN}', '{"t":"nope"}'):
            await ws.write_message(bad)
            reply = json.loads(await ws.read_message())
            assert reply["t"] == "error" and reply["msg"]
        assert self.received == []
        ws.close()

    @gen_test
    async def test_a_handler_that_refuses_a_command_reports_why(self):
        ws, _ = await self.connect()
        self.reject = True
        await ws.write_message(json.dumps({"t": "goal", "x": 1, "y": 2}))
        assert json.loads(await ws.read_message()) == {"t": "error", "msg": "goal is outside the map"}
        ws.close()

    @gen_test
    async def test_binary_frames_from_a_client_are_refused(self):
        ws, _ = await self.connect()
        await ws.write_message(b"\x01\x02", binary=True)
        assert json.loads(await ws.read_message())["t"] == "error"
        ws.close()

    @gen_test
    async def test_an_oversized_message_closes_the_connection(self):
        ws, _ = await self.connect()
        await ws.write_message("x" * 200_000)
        assert await ws.read_message() is None            # server closed the socket (None = closed)


class TestBroadcast(_Base):
    @gen_test
    async def test_state_and_map_reach_every_client(self):
        a, _ = await self.connect()
        b, _ = await self.connect()
        self.server.publish_state({"uav": {"x": 1.5}})
        self.server.publish_map(self.sample_map(2))
        for ws in (a, b):
            assert json.loads(await ws.read_message()) == {"t": "state", "uav": {"x": 1.5}}
            assert protocol.unpack_map(await ws.read_message()).header["v"] == 2
        a.close()
        b.close()

    @gen_test
    async def test_nan_in_state_is_sent_as_null(self):
        ws, _ = await self.connect()
        self.server.publish_state({"rtf": float("nan")})
        assert json.loads(await ws.read_message()) == {"t": "state", "rtf": None}
        ws.close()

    @gen_test
    async def test_a_closed_client_is_forgotten_and_the_others_keep_working(self):
        a, _ = await self.connect()
        b, _ = await self.connect()
        a.close()
        await self.io_loop.run_in_executor(None, lambda: None)
        for _ in range(3):
            self.server.publish_state({"n": 1})
        assert json.loads(await b.read_message())["n"] == 1
        b.close()


class TestStickyEvents(_Base):
    @gen_test
    async def test_a_sticky_event_is_replayed_to_a_late_client_and_only_the_newest_of_each_kind(self):
        self.server.publish_event({"t": "plan", "plan": {"valid": True, "n": 1}}, sticky=True)
        self.server.publish_event({"t": "plan", "plan": {"valid": True, "n": 2}}, sticky=True)
        self.server.publish_event({"t": "log", "msg": "not remembered"})          # not sticky
        await self.io_loop.run_in_executor(None, lambda: None)
        ws, _ = await self.connect()
        msg = json.loads(await ws.read_message())
        assert msg == {"t": "plan", "plan": {"valid": True, "n": 2}}
        # nothing else was queued for this client: the next message is whatever we publish now
        self.server.publish_event({"t": "log", "msg": "live"})
        assert json.loads(await ws.read_message()) == {"t": "log", "msg": "live"}
        ws.close()

    @gen_test
    async def test_clearing_a_sticky_plan_with_null_replaces_the_old_one(self):
        self.server.publish_event({"t": "plan", "plan": {"valid": True}}, sticky=True)
        self.server.publish_event({"t": "plan", "plan": None}, sticky=True)
        await self.io_loop.run_in_executor(None, lambda: None)
        ws, _ = await self.connect()
        assert json.loads(await ws.read_message()) == {"t": "plan", "plan": None}
        ws.close()
