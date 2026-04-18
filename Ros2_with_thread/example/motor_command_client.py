#!/usr/bin/env python3
import json
from urllib import request, parse


class GatewayError(Exception):
    pass


class GatewayClient:
    def __init__(self, host="127.0.0.1", port=8080, timeout=2.0):
        self.base_url = f"http://{host}:{port}"
        self.timeout = timeout

    def _post(self, path: str, payload: dict):
        url = self.base_url + path
        data = json.dumps(payload).encode("utf-8")
        req = request.Request(
            url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with request.urlopen(req, timeout=self.timeout) as resp:
                body = resp.read().decode("utf-8")
                result = json.loads(body) if body else {}
        except Exception as e:
            raise GatewayError(f"POST {path} failed: {e}") from e

        if not result.get("ok", False):
            raise GatewayError(result.get("error", f"POST {path} failed"))
        return result

    def _get(self, path: str, params: dict | None = None):
        url = self.base_url + path
        if params:
            url += "?" + parse.urlencode(params)
        try:
            with request.urlopen(url, timeout=self.timeout) as resp:
                body = resp.read().decode("utf-8")
                result = json.loads(body) if body else {}
        except Exception as e:
            raise GatewayError(f"GET {path} failed: {e}") from e

        if not result.get("ok", False):
            raise GatewayError(result.get("error", f"GET {path} failed"))
        return result

    def publish(self, topic: str, msg_type: str, payload: dict):
        return self._post("/publish", {
            "topic": topic,
            "msg_type": msg_type,
            "payload": payload,
        })

    def subscribe(self, topic: str, msg_type: str):
        return self._post("/subscribe", {
            "topic": topic,
            "msg_type": msg_type,
        })

    def get_message(self, topic: str, msg_type: str):
        return self._get("/message", {
            "topic": topic,
            "msg_type": msg_type,
        })


class MotorCommandClient:
    MSG_TYPE = "motor_control_interfaces/msg/MotorCommand"

    MODE_VELOCITY = 0
    MODE_POSITION = 1
    MODE_MOTION = 2
    MODE_ENABLE = 3
    MODE_DISABLE = 4

    def __init__(
        self,
        gateway: GatewayClient,
        topic="/motor_commands",
        default_kp=10.0,
        default_kd=0.2,
        default_velocity=0.0,
    ):
        self.gateway = gateway
        self.topic = topic
        self.default_kp = default_kp
        self.default_kd = default_kd
        self.default_velocity = default_velocity

    def _as_list(self, x, n=None, default=None):
        if isinstance(x, list):
            out = x
        elif x is None:
            out = []
        else:
            out = [x]

        if n is None:
            return out

        if len(out) == 0:
            return [default for _ in range(n)]
        if len(out) < n:
            out = out + [out[-1] if default is None else default] * (n - len(out))
        return out[:n]

    def _publish(
        self,
        joint_name,
        mode,
        position=None,
        velocity=None,
        acceleration=None,
        torque=None,
        kp=None,
        kd=None,
    ):
        joint_name = self._as_list(joint_name)
        n = len(joint_name)
        if n == 0:
            raise ValueError("joint_name cannot be empty")

        mode = self._as_list(mode, n=n, default=self.MODE_MOTION)
        position = [float(x) for x in self._as_list(position, n=n, default=0.0)]
        velocity = [float(x) for x in self._as_list(velocity, n=n, default=self.default_velocity)]
        acceleration = [float(x) for x in self._as_list(acceleration, n=n, default=0.0)]
        torque = [float(x) for x in self._as_list(torque, n=n, default=0.0)]
        kp = [float(x) for x in self._as_list(kp, n=n, default=self.default_kp)]
        kd = [float(x) for x in self._as_list(kd, n=n, default=self.default_kd)]

        payload = {
            "joint_name": [str(x) for x in joint_name],
            "mode": [int(x) for x in mode],
            "position": position,
            "velocity": velocity,
            "acceleration": acceleration,
            "torque": torque,
            "kp": kp,
            "kd": kd,
        }
        return self.gateway.publish(self.topic, self.MSG_TYPE, payload)

    def publish_raw(self, payload: dict):
        return self.gateway.publish(self.topic, self.MSG_TYPE, payload)

    def motion(
        self,
        joint_name,
        position,
        velocity=None,
        kp=None,
        kd=None,
        torque=None,
        acceleration=None,
    ):
        return self._publish(
            joint_name=joint_name,
            mode=self.MODE_MOTION,
            position=position,
            velocity=velocity,
            kp=kp,
            kd=kd,
            torque=torque,
            acceleration=acceleration,
        )

    def position(self, joint_name, position, velocity=None, acceleration=None):
        return self._publish(
            joint_name=joint_name,
            mode=self.MODE_POSITION,
            position=position,
            velocity=velocity,
            acceleration=acceleration,
        )

    def velocity(self, joint_name, velocity):
        return self._publish(
            joint_name=joint_name,
            mode=self.MODE_VELOCITY,
            velocity=velocity,
        )

    def enable(self, joint_name):
        return self._publish(
            joint_name=joint_name,
            mode=self.MODE_ENABLE,
        )

    def disable(self, joint_name):
        return self._publish(
            joint_name=joint_name,
            mode=self.MODE_DISABLE,
        )

    def hold(self, joint_name, position, kp=None, kd=None):
        return self.motion(
            joint_name=joint_name,
            position=position,
            velocity=0.0,
            kp=kp,
            kd=kd,
            torque=0.0,
            acceleration=0.0,
        )

    def subscribe_commands(self):
        return self.gateway.subscribe(self.topic, self.MSG_TYPE)

    def get_last_command(self):
        return self.gateway.get_message(self.topic, self.MSG_TYPE)