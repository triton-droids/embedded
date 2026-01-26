import sys
sys.path.append("./build")  # adjust if you run elsewhere

import control_core_py as cc

ctrl = cc.Controller()
ctrl.configure(["shoulder_pitch", "elbow_pitch", "wrist_roll"])

# Build state (optional; can be empty for now)
s = cc.State()
jd = cc.JointData()
jd.pos = 0.1
jd.vel = 0.0
jd.eff = 0.0
s.joints["shoulder_pitch"] = jd

# Subset commands (independent per joint mode)
desired = {}

c0 = cc.Command()
c0.mode = cc.Mode.Position
c0.position = 0.0
c0.velocity = 0.5
c0.acceleration = 1.0
desired["shoulder_pitch"] = c0

c1 = cc.Command()
c1.mode = cc.Mode.Velocity
c1.velocity = -0.2
desired["elbow_pitch"] = c1

c2 = cc.Command()
c2.mode = cc.Mode.Motion
c2.position = 0.3
c2.velocity = 0.0
c2.torque = 0.0
c2.kp = 40.0
c2.kd = 1.5
desired["wrist_roll"] = c2

out = ctrl.step(s, desired, 0.02)

for jn, cmd in zip(out.joint_name, out.commands):
    print(jn, cmd.mode, "vel=", cmd.velocity, "pos=", cmd.position, "kp=", cmd.kp, "kd=", cmd.kd)

# Step again with empty desired => hold-last
out2 = ctrl.step(s, {}, 0.02)
print("hold-last:", [c.velocity for c in out2.commands])
