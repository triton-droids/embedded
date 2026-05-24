#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import json
import math
import sys
import threading
import time
from pathlib import Path
from queue import Empty, SimpleQueue
from typing import Any

import rclpy
from motor_control_interfaces.msg import MotorCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _add_repo_venv_site_packages() -> None:
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "rosenv"
        if candidate.is_dir():
            site_packages = sorted((candidate / "lib").glob("python*/site-packages"))
            for path in site_packages:
                path_str = str(path)
                if path_str not in sys.path:
                    sys.path.append(path_str)
            return


try:
    import websockets
except ModuleNotFoundError:
    _add_repo_venv_site_packages()
    import websockets

from websockets.asyncio.server import ServerConnection, serve
from websockets.datastructures import Headers
from websockets.http11 import Request, Response


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ROS2 Double Pendulum</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      color: #18212f;
      background: #f6f8fb;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 340px;
      gap: 0;
    }
    main {
      min-width: 0;
      display: flex;
      align-items: stretch;
      justify-content: center;
      padding: 18px;
    }
    canvas {
      width: 100%;
      height: calc(100vh - 36px);
      border: 1px solid #d3dae6;
      background: #ffffff;
      border-radius: 8px;
    }
    aside {
      border-left: 1px solid #d3dae6;
      background: #ffffff;
      padding: 18px;
      overflow: auto;
    }
    h1 {
      margin: 0 0 16px;
      font-size: 20px;
      line-height: 1.2;
    }
    .status {
      display: flex;
      align-items: center;
      gap: 8px;
      margin-bottom: 18px;
      font-size: 14px;
    }
    .dot {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: #b42318;
    }
    .dot.ok { background: #0e8f55; }
    section {
      border-top: 1px solid #e6eaf0;
      padding-top: 16px;
      margin-top: 16px;
    }
    label {
      display: block;
      font-size: 13px;
      font-weight: 650;
      margin: 14px 0 6px;
    }
    input[type="range"] { width: 100%; }
    input[type="number"] {
      width: 100%;
      padding: 8px 10px;
      border: 1px solid #cbd3df;
      border-radius: 6px;
      font: inherit;
    }
    .row {
      display: grid;
      grid-template-columns: 1fr 74px;
      gap: 10px;
      align-items: center;
    }
    .buttons {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 14px;
    }
    button {
      border: 1px solid #b9c3d2;
      background: #f8fafc;
      color: #18212f;
      border-radius: 6px;
      padding: 9px 10px;
      font: inherit;
      font-weight: 650;
      cursor: pointer;
    }
    button.primary {
      border-color: #1d6fb8;
      background: #1d6fb8;
      color: #ffffff;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }
    td {
      padding: 8px 0;
      border-bottom: 1px solid #edf1f6;
    }
    td:last-child {
      text-align: right;
      font-variant-numeric: tabular-nums;
    }
    @media (max-width: 820px) {
      body { grid-template-columns: 1fr; }
      canvas { height: 62vh; }
      aside { border-left: 0; border-top: 1px solid #d3dae6; }
    }
  </style>
</head>
<body>
  <main><canvas id="scene"></canvas></main>
  <aside>
    <h1>ROS2 Double Pendulum</h1>
    <div class="status"><span id="dot" class="dot"></span><span id="status">Disconnected</span></div>

    <section>
      <table>
        <tbody>
          <tr><td>test_joint</td><td id="j1">0.000 rad</td></tr>
          <tr><td>test_joint2</td><td id="j2">0.000 rad</td></tr>
          <tr><td>Last state</td><td id="age">-</td></tr>
          <tr><td>Last command</td><td id="cmd">-</td></tr>
        </tbody>
      </table>
    </section>

    <section>
      <label for="theta1">test_joint target</label>
      <div class="row">
        <input id="theta1" type="range" min="-3.14" max="3.14" step="0.01" value="0">
        <input id="theta1n" type="number" min="-3.14" max="3.14" step="0.01" value="0">
      </div>
      <label for="theta2">test_joint2 target</label>
      <div class="row">
        <input id="theta2" type="range" min="-3.14" max="3.14" step="0.01" value="0">
        <input id="theta2n" type="number" min="-3.14" max="3.14" step="0.01" value="0">
      </div>
      <label for="speed">Command speed rad/s</label>
      <input id="speed" type="number" min="0" max="10" step="0.1" value="1.0">
      <div class="buttons">
        <button id="enable">Enable</button>
        <button id="disable">Disable</button>
      </div>
      <div class="buttons">
        <button id="send" class="primary">Send Position</button>
        <button id="zero">Zero</button>
      </div>
    </section>
  </aside>

  <script>
    const joints = ["test_joint", "test_joint2"];
    const state = { test_joint: 0, test_joint2: 0, stamp: 0 };
    const canvas = document.getElementById("scene");
    const ctx = canvas.getContext("2d");
    const dot = document.getElementById("dot");
    const statusEl = document.getElementById("status");
    const j1 = document.getElementById("j1");
    const j2 = document.getElementById("j2");
    const age = document.getElementById("age");
    const cmd = document.getElementById("cmd");
    let ws;

    function resize() {
      const scale = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      canvas.width = Math.floor(rect.width * scale);
      canvas.height = Math.floor(rect.height * scale);
      ctx.setTransform(scale, 0, 0, scale, 0, 0);
    }

    function connect() {
      const scheme = location.protocol === "https:" ? "wss" : "ws";
      ws = new WebSocket(`${scheme}://${location.host}/ws`);
      ws.onopen = () => {
        dot.classList.add("ok");
        statusEl.textContent = "Connected";
      };
      ws.onclose = () => {
        dot.classList.remove("ok");
        statusEl.textContent = "Disconnected";
        setTimeout(connect, 800);
      };
      ws.onmessage = event => {
        const msg = JSON.parse(event.data);
        if (msg.type === "command_ack") {
          cmd.textContent = `${msg.mode} ok`;
          return;
        }
        if (msg.type !== "joint_state") return;
        for (const joint of msg.joints || []) {
          if (joint.name in state) state[joint.name] = joint.position_rad || 0;
        }
        state.stamp = msg.stamp_unix_s || Date.now() / 1000;
      };
    }

    function sendCommand(mode, position) {
      if (!ws || ws.readyState !== WebSocket.OPEN) {
        cmd.textContent = "not connected";
        return;
      }
      const payload = { type: "command", mode, joint_names: joints };
      if (mode === "position") {
        payload.position_rad = position;
        payload.velocity_radps = [Number(speed.value), Number(speed.value)];
        payload.kp = [40, 40];
        payload.kd = [1.5, 1.5];
      }
      ws.send(JSON.stringify(payload));
      cmd.textContent = `${mode} sent`;
    }

    function bindPair(rangeId, numberId) {
      const range = document.getElementById(rangeId);
      const number = document.getElementById(numberId);
      range.addEventListener("input", () => number.value = range.value);
      number.addEventListener("input", () => range.value = number.value);
      return number;
    }

    const theta1 = bindPair("theta1", "theta1n");
    const theta2 = bindPair("theta2", "theta2n");
    const speed = document.getElementById("speed");
    document.getElementById("enable").onclick = () => sendCommand("enable");
    document.getElementById("disable").onclick = () => sendCommand("disable");
    document.getElementById("send").onclick = () => sendCommand("position", [Number(theta1.value), Number(theta2.value)]);
    document.getElementById("zero").onclick = () => {
      theta1.value = theta2.value = "0";
      document.getElementById("theta1").value = "0";
      document.getElementById("theta2").value = "0";
      sendCommand("position", [0, 0]);
    };

    function draw() {
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      const margin = 28;
      const cx = rect.width / 2;
      const cy = rect.height / 2;
      const len = Math.max(24, Math.min(rect.width - margin * 2, rect.height - margin * 2) / 4);
      const a1 = state.test_joint;
      const a2 = state.test_joint2;
      const x1 = cx + Math.sin(a1) * len;
      const y1 = cy + Math.cos(a1) * len;
      const x2 = x1 + Math.sin(a1 + a2) * len;
      const y2 = y1 + Math.cos(a1 + a2) * len;

      ctx.lineWidth = 2;
      ctx.strokeStyle = "#d7dde7";
      for (let r = len; r <= len * 2.05; r += len) {
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
        ctx.stroke();
      }

      ctx.lineCap = "round";
      ctx.lineWidth = 12;
      ctx.strokeStyle = "#245f95";
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(x1, y1);
      ctx.stroke();
      ctx.strokeStyle = "#b65a2b";
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();

      for (const [x, y, fill] of [[cx, cy, "#18212f"], [x1, y1, "#1d6fb8"], [x2, y2, "#b65a2b"]]) {
        ctx.beginPath();
        ctx.fillStyle = fill;
        ctx.arc(x, y, 13, 0, Math.PI * 2);
        ctx.fill();
      }

      j1.textContent = `${state.test_joint.toFixed(3)} rad`;
      j2.textContent = `${state.test_joint2.toFixed(3)} rad`;
      age.textContent = state.stamp ? `${Math.max(0, Date.now() / 1000 - state.stamp).toFixed(2)} s` : "-";
      requestAnimationFrame(draw);
    }

    window.addEventListener("resize", resize);
    resize();
    connect();
    draw();
  </script>
</body>
</html>
"""


class DoublePendulumWebsocketNode(Node):
    def __init__(self) -> None:
        super().__init__("double_pendulum_websocket_node")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("joint_names", ["test_joint", "test_joint2"])

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        names_param = self.get_parameter("joint_names").value
        self.joint_names = [str(name) for name in names_param]

        self._ws_clients: set[ServerConnection] = set()
        self._loop = asyncio.new_event_loop()
        self._server: Any | None = None
        self._command_queue: SimpleQueue[dict[str, Any]] = SimpleQueue()
        self._command_pub = self.create_publisher(MotorCommand, "motor_commands", 10)
        self._joint_state_sub = self.create_subscription(
            JointState,
            "joint_states",
            self._joint_state_callback,
            10,
        )
        self._command_timer = self.create_timer(0.02, self._drain_command_queue)

        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"Double pendulum UI listening at http://{self.host}:{self.port}")

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._start_server())
        self._loop.run_forever()

    async def _start_server(self) -> None:
        self._server = await serve(
            self._handle_websocket,
            self.host,
            self.port,
            process_request=self._process_http_request,
        )

    def _process_http_request(
        self,
        connection: ServerConnection,
        request: Request,
    ) -> Response | None:
        _ = connection
        if request.path == "/ws":
            return None

        if request.path not in ("/", "/index.html"):
            body = b"Not found\n"
            headers = Headers()
            headers["Content-Type"] = "text/plain"
            headers["Content-Length"] = str(len(body))
            return Response(404, "Not Found", headers, body)

        body = HTML_PAGE.encode("utf-8")
        headers = Headers()
        headers["Content-Type"] = "text/html; charset=utf-8"
        headers["Content-Length"] = str(len(body))
        return Response(200, "OK", headers, body)

    async def _handle_websocket(self, connection: ServerConnection) -> None:
        if connection.request is not None and connection.request.path != "/ws":
            await connection.close(code=1008, reason="use /ws")
            return

        self._ws_clients.add(connection)
        try:
            async for message in connection:
                if isinstance(message, str):
                    self._handle_ws_message(message)
        finally:
            self._ws_clients.discard(connection)

    def _handle_ws_message(self, message: str) -> None:
        if not message:
            return
        try:
            payload = json.loads(message)
        except json.JSONDecodeError:
            return
        if payload.get("type") != "command":
            return
        self._command_queue.put(payload)

    def _drain_command_queue(self) -> None:
        for _ in range(100):
            try:
                payload = self._command_queue.get_nowait()
            except Empty:
                return
            command_info = self._publish_command_payload(payload)
            if command_info is not None:
                self.get_logger().info(
                    f"WebSocket command published: {command_info['mode']} "
                    f"{', '.join(command_info['joint_names'])}"
                )
                asyncio.run_coroutine_threadsafe(
                    self._broadcast(
                        {
                            "type": "command_ack",
                            "mode": command_info["mode"],
                            "joint_names": command_info["joint_names"],
                            "stamp_unix_s": time.time(),
                        }
                    ),
                    self._loop,
                )

    def _publish_command_payload(self, payload: dict[str, Any]) -> dict[str, Any] | None:
        joint_names = [str(name) for name in payload.get("joint_names", self.joint_names)]
        if not joint_names:
            return None

        mode_name = str(payload.get("mode", "position")).lower()
        mode_by_name = {
            "enable": MotorCommand.MODE_ENABLE,
            "disable": MotorCommand.MODE_DISABLE,
            "velocity": MotorCommand.MODE_VELOCITY,
            "position": MotorCommand.MODE_POSITION,
            "motion": MotorCommand.MODE_MOTION,
            "mit": MotorCommand.MODE_MOTION,
        }
        mode = mode_by_name.get(mode_name)
        if mode is None:
            return None

        msg = MotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = joint_names
        msg.mode = [mode]
        count = len(joint_names)
        msg.position = self._numeric_list(payload.get("position_rad", []), count)
        msg.velocity = self._numeric_list(payload.get("velocity_radps", []), count)
        msg.acceleration = self._numeric_list(payload.get("acceleration_radps2", []), count)
        msg.torque = self._numeric_list(payload.get("torque_nm", []), count)
        msg.kp = self._numeric_list(payload.get("kp", []), count)
        msg.kd = self._numeric_list(payload.get("kd", []), count)
        self._command_pub.publish(msg)
        return {"mode": mode_name, "joint_names": joint_names}

    def _numeric_list(self, value: Any, count: int) -> list[float]:
        if value is None:
            return []
        if isinstance(value, (int, float)):
            return [float(value)] * count
        if not isinstance(value, list):
            return []
        result = [float(item) for item in value[:count]]
        if len(result) == 1 and count > 1:
            return result * count
        return result

    def _joint_state_callback(self, msg: JointState) -> None:
        now = time.time()
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp <= 0.0:
            stamp = now

        joints = []
        wanted = set(self.joint_names)
        for index, name in enumerate(msg.name):
            if name not in wanted:
                continue
            joints.append(
                {
                    "name": name,
                    "position_rad": self._finite_value(msg.position, index),
                    "velocity_radps": self._finite_value(msg.velocity, index),
                    "effort_nm": self._finite_value(msg.effort, index),
                }
            )

        payload = {
            "type": "joint_state",
            "stamp_unix_s": stamp,
            "server_unix_s": now,
            "joints": joints,
        }
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    async def _broadcast(self, payload: dict[str, Any]) -> None:
        if not self._ws_clients:
            return
        data = json.dumps(payload, separators=(",", ":"))
        websockets.broadcast(self._ws_clients, data)

    @staticmethod
    def _finite_value(values: list[float] | tuple[float, ...], index: int) -> float:
        value = float(values[index]) if index < len(values) else 0.0
        return value if math.isfinite(value) else 0.0

    def destroy_node(self) -> None:
        async def shutdown() -> None:
            for client in list(self._ws_clients):
                await client.close()
            if self._server is not None:
                self._server.close()
                await self._server.wait_closed()

        future = asyncio.run_coroutine_threadsafe(shutdown(), self._loop)
        try:
            future.result(timeout=1.0)
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DoublePendulumWebsocketNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
