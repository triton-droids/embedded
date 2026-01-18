# vla_test.py
# 本地 FastAPI：英文指令 -> (优先本地规则秒回) -> 失败再调用 Spark X1.5 HTTP -> 返回 ONLY 一个 ROS2 service call 命令
#
# 特性：
# - /compile 只返回 {"command": "..."}
# - 本地规则 local_compile：enable/disable/status/position/velocity/current/torque 直接生成命令（几乎 0ms）
# - LLM 兜底：遇到复杂指令才请求 Spark（可能 3-8s）
# - 稳健 extract/validate：允许 joint_name 无引号/单引号/双引号
# - 出错返回 400（带 raw 前 2000 字符）
# - /execute 可选执行命令（带白名单校验）
#
# 依赖：
#   pip install fastapi uvicorn requests pydantic

import os
import re
import math
import time
import subprocess
from dataclasses import dataclass
from typing import Optional, Dict, List

import requests
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# -----------------------------
# 0) 常量
# -----------------------------
ALLOWED_JOINTS = {"shoulder_pitch", "left_knee", "left_ankle", "right_knee", "right_ankle"}
CMD_PREFIX = "ros2 service call /robstride_joint_control"

# -----------------------------
# 1) 最短稳定 SYSTEM PROMPT（仅用于 LLM 兜底）
# -----------------------------
SYSTEM_PROMPT = (
"Only output ONE ros2 command, no extra text.\n"
"Must start with: ros2 service call /robstride_joint_control\n"
"joint whitelist: shoulder_pitch,left_knee,left_ankle,right_knee,right_ankle else ERROR: unknown joint_name\n"
"command_type: enable=0 disable=1 velocity=2 position_pp=3 current=4 motion=5 read_status=6 else ERROR: unknown command_type\n"
"missing numbers=0.0; deg->rad (deg*pi/180); decimal floats only.\n"
"Output exactly ONE line:\n"
"ros2 service call /robstride_joint_control motor_control_interfaces/srv/RobStrideJointControl \\ "
"\"{joint_name: '<JOINT>', command_type: <INT>, position: <FLOAT>, velocity: <FLOAT>, torque: <FLOAT>, iq: <FLOAT>, id: <FLOAT>, acceleration: <FLOAT>, kp: <FLOAT>, kd: <FLOAT>}\""
)

# -----------------------------
# 2) Spark X1.5 HTTP client
# -----------------------------
@dataclass
class SparkHttpConfig:
    api_password: str  # Bearer token (APIpassword)
    endpoint: str = "https://spark-api-open.xf-yun.com/v2/chat/completions"
    model: str = "spark-x"
    temperature: float = 0.0
    max_tokens: int = 128
    thinking: str = "disabled"  # enabled/disabled/auto
    timeout_s: int = 60


SESSION = requests.Session()
SESSION.trust_env = False  # 避免环境变量代理/证书等带来的不可控慢路径


def spark_chat(cfg: SparkHttpConfig, messages: List[Dict[str, str]]) -> str:
    if not cfg.api_password:
        raise RuntimeError("Missing SPARK_API_PASSWORD (APIpassword).")

    headers = {
        "Authorization": f"Bearer {cfg.api_password}",
        "Content-Type": "application/json",
    }
    payload = {
        "model": cfg.model,
        "user": "local-api",
        "stream": False,
        "messages": messages,
        "temperature": cfg.temperature,
        "max_tokens": cfg.max_tokens,
        "thinking": {"type": cfg.thinking},
    }

    r = SESSION.post(cfg.endpoint, headers=headers, json=payload, timeout=cfg.timeout_s)
    if r.status_code != 200:
        raise RuntimeError(f"Spark HTTP {r.status_code}: {r.text}")

    data = r.json()
    return data["choices"][0]["message"]["content"]


CFG = SparkHttpConfig(
    api_password=os.getenv("SPARK_API_PASSWORD", ""),
    endpoint=os.getenv("SPARK_ENDPOINT", "https://spark-api-open.xf-yun.com/v2/chat/completions"),
    model=os.getenv("SPARK_MODEL", "spark-x"),
    temperature=float(os.getenv("SPARK_TEMPERATURE", "0.0")),
    max_tokens=int(os.getenv("SPARK_MAX_TOKENS", "128")),
    thinking=os.getenv("SPARK_THINKING", "disabled"),
    timeout_s=int(os.getenv("SPARK_TIMEOUT_S", "60")),
)

# -----------------------------
# 3) 本地规则编译（秒回）
# -----------------------------
def local_compile(instruction: str) -> Optional[str]:
    s = instruction.strip()
    sl = s.lower()

    # 找 joint
    joint = None
    for j in ALLOWED_JOINTS:
        if j in s:
            joint = j
            break
    if joint is None:
        return None

    def emit(
        command_type: int,
        position=0.0,
        velocity=0.0,
        torque=0.0,
        iq=0.0,
        id_=0.0,
        acceleration=0.0,
        kp=0.0,
        kd=0.0,
    ) -> str:
        # 单行输出：更短、更快、更不容易截断
        return (
            "ros2 service call /robstride_joint_control motor_control_interfaces/srv/RobStrideJointControl \\ "
            f"\"{{joint_name: '{joint}', command_type: {int(command_type)}, "
            f"position: {float(position)}, velocity: {float(velocity)}, torque: {float(torque)}, "
            f"iq: {float(iq)}, id: {float(id_)}, acceleration: {float(acceleration)}, kp: {float(kp)}, kd: {float(kd)}}}\""
        )

    # enable / disable / status
    if re.search(r"\b(enable|power on|turn on)\b", sl):
        return emit(0)
    if re.search(r"\b(disable|power off|turn off)\b", sl):
        return emit(1)
    if re.search(r"\b(read status|query status|get status|status)\b", sl):
        return emit(6)

    # current iq / id
    m = re.search(r"\biq\b\s*(?:to|=)?\s*([-+]?\d+(\.\d+)?)", sl)
    if m:
        return emit(4, iq=float(m.group(1)))
    m = re.search(r"\bid\b\s*(?:to|=)?\s*([-+]?\d+(\.\d+)?)", sl)
    if m:
        return emit(4, id_=float(m.group(1)))

    # velocity (rad/s)
    m = re.search(r"\b(velocity|speed)\b.*?([-+]?\d+(\.\d+)?)", sl)
    if m:
        return emit(2, velocity=float(m.group(2)))

    # position (rad/deg)
    m = re.search(
        r"\b(move|go|set|position|angle)\b.*?\bto\b\s*([-+]?\d+(\.\d+)?)\s*(deg|degree|rad|radian)?",
        sl,
    )
    if m:
        val = float(m.group(2))
        unit = (m.group(4) or "rad")
        if unit.startswith("deg"):
            val = val * math.pi / 180.0
        return emit(3, position=val)

    # torque -> 用 motion=5 作为通用载体
    m = re.search(r"\btorque\b.*?([-+]?\d+(\.\d+)?)", sl)
    if m:
        return emit(5, torque=float(m.group(1)))

    # 规则解析不了就交给 LLM
    return None


# -----------------------------
# 4) 抽取/校验（LLM 兜底输出时用）
# -----------------------------
def extract_command(raw: str) -> str:
    raw = (raw or "").strip()

    if raw.startswith("ERROR:"):
        return raw.splitlines()[0].strip()

    idx = raw.find(CMD_PREFIX)
    if idx < 0:
        for line in raw.splitlines():
            if line.strip().startswith(CMD_PREFIX):
                idx = raw.find(line)
                break
    if idx < 0:
        raise ValueError("Model output does not contain the required ros2 command prefix.")

    tail = raw[idx:].strip()

    # 尽量截到最后一个 '}"'
    end = tail.rfind('}"')
    if end != -1:
        tail = tail[: end + 2].strip()

    # 防止模型输出过长
    if len(tail) > 4000:
        tail = tail[:4000].strip()

    if not tail.startswith(CMD_PREFIX):
        raise ValueError("Extracted text does not start with required prefix.")
    return tail


def validate_command(cmd: str) -> None:
    if cmd.startswith("ERROR:"):
        return
    if not cmd.startswith(CMD_PREFIX):
        raise ValueError("Command prefix mismatch.")

    # joint_name 允许无引号/单引号/双引号
    m = re.search(r"joint_name\s*:\s*['\"]?([a-zA-Z0-9_]+)['\"]?", cmd)
    if not m:
        raise ValueError("joint_name not found in command.")

    joint = m.group(1)
    if joint not in ALLOWED_JOINTS:
        raise ValueError(f"joint_name '{joint}' not in whitelist.")


def build_messages(user_instruction: str) -> List[Dict[str, str]]:
    return [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": user_instruction.strip()},
    ]


# -----------------------------
# 5) FastAPI
# -----------------------------
app = FastAPI(title="Spark RobStride Local API (local-first)")

class CompileReq(BaseModel):
    instruction: str

class CompileResp(BaseModel):
    command: str

class ExecuteReq(BaseModel):
    instruction: Optional[str] = None
    command: Optional[str] = None

class ExecuteResp(BaseModel):
    command: str
    stdout: str
    stderr: str
    returncode: int


@app.get("/health")
def health():
    return {
        "ok": True,
        "mode": "local-first",
        "allowed_joints": sorted(list(ALLOWED_JOINTS)),
        "spark": {
            "endpoint": CFG.endpoint,
            "model": CFG.model,
            "thinking": CFG.thinking,
            "temperature": CFG.temperature,
            "max_tokens": CFG.max_tokens,
            "timeout_s": CFG.timeout_s,
            "enabled": bool(CFG.api_password),
        },
    }


@app.post("/compile", response_model=CompileResp)
def compile_cmd(req: CompileReq):
    t0 = time.time()

    # 1) 本地规则优先（秒回）
    local_cmd = local_compile(req.instruction)
    if local_cmd is not None:
        validate_command(local_cmd)
        t1 = time.time()
        print(f"[timing] local=1 total={t1-t0:.3f}s")
        return CompileResp(command=local_cmd)

    # 2) LLM 兜底
    msgs = build_messages(req.instruction)
    t1 = time.time()

    raw = spark_chat(CFG, msgs)
    t2 = time.time()

    try:
        cmd = extract_command(raw)
        validate_command(cmd)
    except Exception as e:
        snippet = (raw or "")[:2000].replace("\n", "\\n")
        raise HTTPException(status_code=400, detail=f"{e}. raw[0:2000]={snippet}")

    t3 = time.time()
    print(f"[timing] build={t1-t0:.3f}s spark={t2-t1:.3f}s parse+check={t3-t2:.3f}s total={t3-t0:.3f}s")
    return CompileResp(command=cmd)


@app.post("/execute", response_model=ExecuteResp)
def execute(req: ExecuteReq):
    if not req.command and not req.instruction:
        raise HTTPException(status_code=400, detail="Provide either 'command' or 'instruction'.")

    if req.command:
        cmd = req.command.strip()
    else:
        # 也先走本地规则
        local_cmd = local_compile(req.instruction or "")
        if local_cmd is not None:
            cmd = local_cmd
        else:
            raw = spark_chat(CFG, build_messages(req.instruction or ""))
            cmd = extract_command(raw)

    try:
        validate_command(cmd)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    p = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return ExecuteResp(
        command=cmd,
        stdout=p.stdout,
        stderr=p.stderr,
        returncode=p.returncode,
    )
