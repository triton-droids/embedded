"""Drive and record the mode-manager scenario: start -> ready -> walk -> fall -> get up -> walk again -> stop.

Needs a running sim with sim_time:=true share_state:=true walk_cmd_vel_topic:=/ball_chase/cmd_vel
mode_manager:=true (run_scenario.sh starts it). Drives on simulated time: walks with /ball_chase/cmd_vel,
pushes with /sim/push, and records the kinematic state (state_shm, 30 fps simulated time),
/mode_manager/state, /walk/status and the scenario phase to OUT.npz + OUT.json + OUT.mjb for
render_scenario.py. Stops early when the mode manager reaches fault or estop.

  python3 record_scenario.py OUT.npz [WALL_LIMIT_S]
"""
import json
import shutil
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String

from humanoid_sim.state_shm import StateReader

LATCHED = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
WALK_V = 0.08          # m/s (0.04 m per 0.5 s step)
WALK_S = 4.0           # sim seconds of walking before the push and after the get-up
SETTLE_S = 2.0         # sim seconds standing before walking (ready) and after getting up
STOP_S = 4.0           # sim seconds standing at the end
PUSH = [150.0, 0.0, 0.0, 0.3]
FRAME_NS = int(1e9 / 30)


class Scenario(Node):
    def __init__(self):
        super().__init__('scenario', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.vel = self.create_publisher(TwistStamped, '/ball_chase/cmd_vel', 2)
        self.push = self.create_publisher(Float64MultiArray, '/sim/push', 10)
        self.create_subscription(String, '/mode_manager/state', self._on_mode, LATCHED)
        self.create_subscription(String, '/walk/status', self._on_walk, 10)
        self.mode, self.walk = None, {}
        self.modes, self.walks, self.phases = [], [], []
        self.phase, self.phase_t = None, None

    def now(self):
        return self.get_clock().now().nanoseconds

    def log(self, text):
        print(f'sim {self.now() * 1e-9:9.2f}  {text}', flush=True)

    def _on_mode(self, msg):
        d = json.loads(msg.data)
        if d['mode'] != self.mode:
            self.mode = d['mode']
            self.modes.append((self.now(), d['mode'], d['reason'], d['getup_attempts']))
            self.log(f"MODE {d['mode']:8s} {d['reason']}")

    def _on_walk(self, msg):
        d = json.loads(msg.data)
        self.walk = d
        # sim_assist: the TEST-ONLY simulator assist ("Aid of Magic", walking_controller recovery_sim_assist).
        key = (d.get('state'), d.get('recovery'), bool(d.get('sim_assist')))
        if not self.walks or (self.walks[-1][1], self.walks[-1][2], self.walks[-1][4]) != key:
            self.walks.append((self.now(), d.get('state'), d.get('recovery'), d.get('steps'), key[2]))

    def set_phase(self, name):
        self.phase, self.phase_t = name, self.now()
        self.phases.append((self.now(), name))
        self.log(f'PHASE {name}')

    def elapsed(self):
        return (self.now() - self.phase_t) * 1e-9

    def send(self, v):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = v
        self.vel.publish(m)

    def tick(self):
        """Advance the scenario; returns False when done."""
        mode = self.mode
        if self.phase is None:
            self.set_phase('start')
        elif self.phase == 'start' and mode == 'stand':
            self.set_phase('ready')
        elif self.phase == 'ready' and self.elapsed() >= SETTLE_S:
            self.set_phase('walk')
        elif self.phase == 'walk' and self.elapsed() >= WALK_S:
            self.push.publish(Float64MultiArray(data=PUSH))
            self.set_phase('fall')
        elif self.phase == 'fall' and mode == 'getup':
            self.set_phase('getup')
        elif self.phase == 'getup' and mode == 'stand':
            self.set_phase('settle')
        elif self.phase == 'settle' and self.elapsed() >= SETTLE_S:
            self.set_phase('walk_again')
        elif self.phase == 'walk_again' and self.elapsed() >= WALK_S:
            self.set_phase('stop')
        elif self.phase == 'stop' and self.elapsed() >= STOP_S:
            return False
        if mode in ('fault', 'estop') and self.phase not in ('start',):
            self.log(f'aborted in mode {mode}')
            return False
        # Velocity: only while walking; zero otherwise (a zero command stands the walking controller).
        self.send(WALK_V if self.phase in ('walk', 'walk_again') else 0.0)
        return True


def main():
    out = sys.argv[1]
    wall_limit = float(sys.argv[2]) if len(sys.argv) > 2 else 2400.0
    rclpy.init()
    node = Scenario()
    reader = StateReader(timeout_s=300)
    rows, last_frame = [], None
    start = time.monotonic()
    next_tick = start
    running = True
    while running and time.monotonic() - start < wall_limit:
        rclpy.spin_once(node, timeout_sec=0.01)
        rec = reader.read()
        if rec is not None and (last_frame is None or rec[0] - last_frame >= FRAME_NS):
            last_frame = rec[0]
            rows.append((rec[0], rec[1].copy(), rec[2].copy(), rec[3].copy()))
        if time.monotonic() >= next_tick:
            next_tick += 0.05
            if node.now() > 0:
                running = node.tick()
    if running:
        node.log('wall limit reached')
    np.savez_compressed(out, stamp=np.array([r[0] for r in rows]), qpos=np.array([r[1] for r in rows]),
                        mocap_pos=np.array([r[2] for r in rows]), mocap_quat=np.array([r[3] for r in rows]))
    with open(out.replace('.npz', '.json'), 'w') as f:
        json.dump({'modes': node.modes, 'walks': node.walks, 'phases': node.phases}, f)
    shutil.copy(reader.model_path, out.replace('.npz', '.mjb'))
    node.log(f'saved {len(rows)} frames')


if __name__ == '__main__':
    main()
