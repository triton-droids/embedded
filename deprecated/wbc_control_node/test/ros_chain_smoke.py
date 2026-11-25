import collections
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from motor_control_interfaces.msg import MotorCommand

os.environ['ROS_LOG_DIR'] = '/tmp/wbc-control-ros-logs'
os.environ['ROS_DOMAIN_ID'] = '173'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
rclpy.init()
node = Node('wbc_integration_probe')
counts = collections.Counter()
latest = {}
motion = collections.Counter()
sources = set()
matched = set()
events = []
foot_samples = []
odom_samples = []
previous = {}
gaps = {}
previous_running = {}
gaps_running = {}
def sample(key, msg):
    counts[key] += 1
    if key in ('imu','joints'):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        received = time.monotonic()
        buckets = [(previous, gaps)]
        if latest.get('wbc_status', {}).get('state') == 'running':
            buckets.append((previous_running, gaps_running))
        for history, metrics in buckets:
            if key in history:
                old_stamp, old_received = history[key]
                metric = metrics.setdefault(key, {'max_source_gap_s':0., 'max_receipt_gap_s':0., 'max_age_s':0.})
                metric['max_source_gap_s'] = max(metric['max_source_gap_s'],stamp-old_stamp)
                metric['max_receipt_gap_s'] = max(metric['max_receipt_gap_s'],received-old_received)
                metric['max_age_s'] = max(metric['max_age_s'],time.time()-stamp)
            history[key] = (stamp,received)
    if key == 'feet':
        foot_samples.append({'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                             'height': list(msg.position), 'force': list(msg.effort)})
    if key == 'odom':
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        odom_samples.append({'roll_deg': math.degrees(roll), 'pitch_deg': math.degrees(pitch), 'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                             'position': [p.x, p.y, p.z]})
    if key in ('wbc_status', 'sim_status', 'imu_status', 'policy_status'):
        try: latest[key] = json.loads(msg.data)
        except Exception: latest[key] = msg.data
    elif key == 'estop': latest[key] = msg.data
    elif key in ('wbc', 'motor'):
        now = time.time()
        source = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        expiry = msg.valid_until.sec + msg.valid_until.nanosec * 1e-9
        events.append({'key':key,'time':now,'mode':list(msg.mode),
                       'age_ms':(now-source)*1000,'remaining_ms':(expiry-now)*1000})
        if list(msg.mode) and all(x == MotorCommand.MODE_MOTION for x in msg.mode):
            motion[key] += 1
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if key == 'wbc':
                assert all(x >= 0 for x in list(msg.kp) + list(msg.kd))
                sources.add(stamp)
            elif any(abs(stamp - source) < 1000 for source in sources):
                matched.add(stamp)
stream = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
for key, topic, cls in [
    ('imu','/imu/data_raw',Imu), ('joints','/joint_states',JointState),
    ('feet','/sim/ground_truth/feet',JointState),
    ('odom','/sim/ground_truth/odom',Odometry),
    ('policy_status','/policy/status',String),
    ('policy','/policy/desired_motor_subset',MotorCommand),
    ('wbc','/wbc/desired_motor_subset',MotorCommand), ('motor','/motor_commands',MotorCommand),
    ('wbc_status','/wbc/status',String), ('sim_status','/sim/status',String),
    ('imu_status','/imu/status',String),
]:
    node.create_subscription(cls, topic, lambda msg, k=key: sample(k,msg), stream)
node.create_subscription(Bool, '/safety/estop', lambda msg: sample('estop',msg),
    QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
walking = '--walking' in sys.argv
hold_first = '--motion' not in sys.argv and not walking
with_viewer = '--viewer' in sys.argv
suffix = 'walking' if walking else ('standing' if hold_first else 'motion')
log = Path('/tmp/wbc-live-' + suffix + '.log').open('w')
command = ['ros2','launch','wbc_control_node','rl_wbc_sim.launch.py',
           'use_viewer:=' + str(with_viewer).lower(),'auto_start:=true', 'hold_first_frame:=' + str(hold_first).lower(),
           'ros_domain_id:=173','web_port:=18083',
           'wbc_reference_end_behavior:=' + ('loop' if walking else 'hold_last_frame')]
process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    end = time.monotonic()+float(os.environ.get('WBC_PROBE_SECONDS', '20'))
    while time.monotonic()<end and process.poll() is None:
        rclpy.spin_once(node,timeout_sec=.05)
finally:
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try: process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGKILL)
        process.wait()
    log.close()
    node.destroy_node()
    rclpy.shutdown()
result = {'counts':dict(counts),'motion_counts':dict(motion),
          'cpp_forwarded_wbc_sources':len(matched),'latest':latest,'gaps':gaps,
          'gaps_running':gaps_running}
airborne_landings = []
for i in range(2):
    begin_air = None
    foot_events = []
    for sample in foot_samples:
        if len(sample['height']) != 2 or len(sample['force']) != 2:
            continue
        air = (sample['height'][i] > .01 and sample['force'][i] < 2.
               and sample['force'][1 - i] > 20.)
        if air and begin_air is None:
            begin_air = sample['stamp']
        elif not air and begin_air is not None and sample['force'][i] > 5.:
            if sample['stamp'] - begin_air >= .04:
                foot_events.append({'lift': begin_air, 'land': sample['stamp']})
            begin_air = None
    airborne_landings.append(foot_events)
displacement = ([odom_samples[-1]['position'][i] - odom_samples[0]['position'][i]
                 for i in range(3)] if odom_samples else [0., 0., 0.])
result['walking'] = {'airborne_landings': airborne_landings, 'displacement_xyz_m': displacement,
                     'max_abs_roll_deg': max((abs(r['roll_deg']) for r in odom_samples), default=0.),
                     'max_abs_pitch_deg': max((abs(r['pitch_deg']) for r in odom_samples), default=0.)}
Path('/tmp/wbc-live-' + suffix + '.json').write_text(json.dumps({**result,'events':events,'feet':foot_samples,'odom':odom_samples},indent=2))
print(json.dumps(result,indent=2))
if (motion['wbc'] < 100 or len(matched) < 50 or latest.get('estop', True)
        or latest.get('wbc_status', {}).get('fault')
        or latest.get('sim_status', {}).get('base_held', True)):
    raise SystemExit(1)
if with_viewer and 'viewer unavailable' in Path('/tmp/wbc-live-' + suffix + '.log').read_text():
    raise SystemExit('Viewer initialization failed; headless fallback is not a GUI pass')
if 'Traceback' in Path('/tmp/wbc-live-' + suffix + '.log').read_text():
    raise SystemExit('Runtime or shutdown traceback in launch log')
if 'C++ verified all motors enabled' not in Path('/tmp/wbc-live-' + suffix + '.log').read_text():
    raise SystemExit('Missing verified-enable handshake')

if walking and (any(len(events) < 2 for events in airborne_landings)
                or math.hypot(*displacement[:2]) < .2
                or not latest.get('policy_status', {}).get('reference_finished')):
    raise SystemExit('Walking not established: need two air/landing cycles per foot, displacement and completed reference')
