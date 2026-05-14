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

    def health(self):
        return self._get("/health")

    def topics(self):
        return self._get("/topics")


class GenericReadClient:
    def __init__(self, gateway: GatewayClient):
        self.gateway = gateway
        self._subscriptions = {}

    def subscribe(self, topic: str, msg_type: str):
        key = (topic, msg_type)
        result = self.gateway.subscribe(topic, msg_type)
        self._subscriptions[key] = True
        return result

    def get_latest(self, topic: str, msg_type: str, auto_subscribe: bool = False):
        key = (topic, msg_type)
        if auto_subscribe and key not in self._subscriptions:
            self.subscribe(topic, msg_type)
        result = self.gateway.get_message(topic, msg_type)
        return result.get("message", None)

    def read(self, topic: str, msg_type: str, auto_subscribe: bool = False):
        return self.get_latest(topic, msg_type, auto_subscribe=auto_subscribe)

    def is_subscribed(self, topic: str, msg_type: str) -> bool:
        return (topic, msg_type) in self._subscriptions

    def get_field(
        self,
        topic: str,
        msg_type: str,
        field_path: str,
        auto_subscribe: bool = False,
        default=None,
    ):
        msg = self.get_latest(topic, msg_type, auto_subscribe=auto_subscribe)
        if msg is None:
            return default

        value = msg
        for part in field_path.split("."):
            if isinstance(value, dict) and part in value:
                value = value[part]
            else:
                return default
        return value

    def wait_for_message(
        self,
        topic: str,
        msg_type: str,
        timeout_sec: float = 3.0,
        poll_interval_sec: float = 0.1,
        auto_subscribe: bool = True,
    ):
        import time

        if auto_subscribe:
            self.subscribe(topic, msg_type)

        start = time.time()
        while time.time() - start < timeout_sec:
            msg = self.get_latest(topic, msg_type, auto_subscribe=False)
            if msg is not None:
                return msg
            time.sleep(poll_interval_sec)

        raise TimeoutError(
            f"No message received for topic={topic}, msg_type={msg_type} within {timeout_sec} s"
        )