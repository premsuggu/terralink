"""The studio's web server: serves the browser app and moves messages over one websocket (`/ws`).

Built on `tornado` (already installed with ROS 2 Humble's Python stack; nothing new to install). It knows nothing about ROS: the ROS node hands it
ready-made messages (`publish_state`, `publish_map`) and receives validated commands through the `on_command` callback. That keeps the whole
message path testable without a simulator (tests/nav/test_studio_server.py).

THREADS. tornado runs in one thread (the "IOLoop thread"); the ROS callbacks run in another. `publish_state` / `publish_map` are safe to call
from any thread: they only schedule the real work on the IOLoop thread with `IOLoop.add_callback`. `on_command` is called ON the IOLoop thread,
so the callback must be quick and must guard shared state itself.

SLOW CLIENTS. A browser that cannot keep up (a busy tab) must not make the server buffer frames forever. While a client is still receiving its
previous frame OF THE SAME KIND (state text / map binary), new frames of that kind for it are dropped (the next one replaces them); commands
from that client are unaffected.

SECURITY. The server binds to 127.0.0.1 by default and uses tornado's same-origin check for websockets, so another web page open in the same
browser cannot connect and drive the robot. WSL2 forwards localhost to Windows, so a Windows browser can still open http://localhost:<port>.
"""
from __future__ import annotations

import asyncio
import logging
from pathlib import Path
from typing import Callable

import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket

from nav.studio.commands import Command, CommandError, parse_command
from nav.studio.protocol import PROTOCOL_VERSION, dumps

log = logging.getLogger("terralink.studio")

# Commands are tiny; anything bigger is a mistake or an attack.
_MAX_CLIENT_MESSAGE_BYTES = 64 * 1024


class _Socket(tornado.websocket.WebSocketHandler):
    """One connected browser tab."""

    def initialize(self, server: "StudioServer") -> None:
        self.server = server
        # True while a frame of that kind is still being written to this client. Kept per kind so a stream of small state frames can
        # never make the (rarer, bigger) map frames get dropped, and vice versa.
        self.busy = {"text": False, "binary": False}

    def open(self) -> None:
        self.server._clients.add(self)
        log.info("studio client connected (%d now)", len(self.server._clients))
        # Everything a new tab needs to draw something at once: who we are, then the newest state and map.
        self.server._send_to(self, dumps({"t": "hello", "protocol": PROTOCOL_VERSION, **self.server.hello}), binary=False, force=True)
        if self.server._last_state is not None:
            self.server._send_to(self, self.server._last_state, binary=False, force=True)
        for text in list(self.server._sticky.values()):
            self.server._send_to(self, text, binary=False, force=True)
        if self.server._last_map is not None:
            self.server._send_to(self, self.server._last_map, binary=True, force=True)

    def on_close(self) -> None:
        self.server._clients.discard(self)
        log.info("studio client left (%d now)", len(self.server._clients))

    def on_message(self, message) -> None:
        if isinstance(message, bytes):            # the protocol has no binary client frames
            self.server._send_to(self, dumps({"t": "error", "msg": "binary frames are not accepted"}), binary=False, force=True)
            return
        try:
            cmd = parse_command(message)
            if cmd.kind == "ping":
                reply = {"t": "pong"}
            else:
                self.server.on_command(cmd)
                reply = None
        except CommandError as exc:
            reply = {"t": "error", "msg": str(exc)}
        if reply is not None:
            self.server._send_to(self, dumps(reply), binary=False, force=True)

    def check_origin(self, origin: str) -> bool:
        # tornado's default: the page's host must equal the Host header (same origin). Kept explicit so it is not changed by accident.
        return super().check_origin(origin)


class StudioServer:
    def __init__(
        self,
        on_command: Callable[[Command], None],
        web_dir: str | Path | None = None,
        host: str = "127.0.0.1",
        port: int = 8765,
        hello: dict | None = None,
        ioloop: tornado.ioloop.IOLoop | None = None,
    ):
        self.on_command = on_command
        self.hello = hello or {}
        self.host, self.port = host, port
        self._ioloop = ioloop or tornado.ioloop.IOLoop.current()
        self._clients: set[_Socket] = set()
        self._last_state: str | None = None
        self._last_map: bytes | None = None
        self._sticky: dict[str, str] = {}      # newest event of each sticky kind (e.g. the current plan), replayed to new tabs
        handlers: list = [(r"/ws", _Socket, {"server": self})]
        if web_dir is not None and Path(web_dir).is_dir():
            handlers.append((r"/(.*)", tornado.web.StaticFileHandler, {"path": str(web_dir), "default_filename": "index.html"}))
        self.app = tornado.web.Application(handlers, websocket_max_message_size=_MAX_CLIENT_MESSAGE_BYTES)
        self._http = None

    # ---- lifecycle --------------------------------------------------------------------------------------------------------------
    def start(self) -> int:
        """Start listening; returns the port actually used (useful when port=0 asks the OS for a free one)."""
        self._http = tornado.httpserver.HTTPServer(self.app)
        self._http.listen(self.port, address=self.host)
        sockets = list(self._http._sockets.values())          # noqa: SLF001 - the only way to learn an OS-chosen port
        if sockets:
            self.port = sockets[0].getsockname()[1]
        log.info("studio server on http://%s:%d", self.host, self.port)
        return self.port

    def stop(self) -> None:
        if self._http is not None:
            self._http.stop()
            self._http = None
        for c in list(self._clients):
            c.close()

    # ---- publishing (any thread) --------------------------------------------------------------------------------------------------
    def publish_state(self, state: dict) -> None:
        """Send a live-state JSON message to every client (a `t:"state"` field is added)."""
        text = dumps({"t": "state", **state})
        self._ioloop.add_callback(self._broadcast, text, False)

    def publish_map(self, frame: bytes) -> None:
        """Send one binary map frame (from `protocol.pack_map`) to every client."""
        self._ioloop.add_callback(self._broadcast, frame, True)

    def publish_event(self, event: dict, sticky: bool = False) -> None:
        """Send any other JSON message (e.g. {"t": "log", ...}). `sticky=True` also remembers the newest one of its kind `t` and replays it to
        tabs that connect later (used for the current plan: a page opened after the plan was made still has to draw it)."""
        text = dumps(event)
        if sticky:
            self._ioloop.add_callback(self._sticky.__setitem__, str(event.get("t")), text)
        self._ioloop.add_callback(self._broadcast, text, False)

    # ---- IOLoop-thread internals --------------------------------------------------------------------------------------------------
    def _broadcast(self, payload, binary: bool) -> None:
        if binary:
            self._last_map = payload
        elif isinstance(payload, str) and payload.startswith('{"t":"state"'):
            self._last_state = payload
        for client in list(self._clients):
            self._send_to(client, payload, binary)

    def _send_to(self, client: _Socket, payload, binary: bool, force: bool = False) -> None:
        kind = "binary" if binary else "text"
        if client.busy[kind] and not force:
            return                     # still sending the previous frame of this kind: drop this one (see module docstring)
        client.busy[kind] = True
        asyncio.ensure_future(self._write(client, payload, binary))

    async def _write(self, client: _Socket, payload, binary: bool) -> None:
        try:
            await client.write_message(payload, binary=binary)
        except tornado.websocket.WebSocketClosedError:
            pass
        finally:
            client.busy["binary" if binary else "text"] = False
