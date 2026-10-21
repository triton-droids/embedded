# Active RobStride CAN path audit

Scope: `python_can_node` -> `NonblockingRobstrideBus` for the configured
RS02/03/04 CAN2.0 private-protocol motors. The standalone legacy SDK/tuner tools
are separate paths; they have not been certified by this audit. Real mechanical
zero remains disabled: simulation completion is not a transmitted type-6 frame.

## Primary references

Official RobStride Product_Information manuals, downloaded 2026-10-21:

- [RS02 2026-07-13](https://raw.githubusercontent.com/RobStride/Product_Information/main/Product%20Literature/CAN2.0/RS02/RS02User%20Manual260713.pdf)
- [RS03 2026-10-13](https://raw.githubusercontent.com/RobStride/Product_Information/main/Product%20Literature/CAN2.0/RS03/RS03User%20Manual261013.pdf)
- [RS04 2026-07-13](https://raw.githubusercontent.com/RobStride/Product_Information/main/Product%20Literature/CAN2.0/RS04/RS04User%20Manual260713.pdf)

The pinned profiles apply to those document revisions. Installed firmware is
unknown and must be matched before setting hardware_verified. In particular,
the installed SDK uses RS03 velocity magnitude 50 while the current official
manual specifies 20 rad/s. This implementation pins 20; it does not silently
infer firmware from model name. Earlier firmware may need a different reviewed
profile. Unsupported models fail before opening the CAN socket.

## Message inventory and findings

| Path | Wire representation | Verification |
| --- | --- | --- |
| Enable | type 3; host in ID bits 8-15; target bits 0-7; eight zero bytes | independent ID/payload vectors; fresh status mode 2 required |
| Disable | type 4; same routing; eight zero bytes | does not set fault-clear byte; fresh status mode 0 verifies stop state |
| Motion / Home | type 1; torque in ID bits 8-23; target bits 0-7; big-endian position/velocity/Kp/Kd | pinned model ranges, independent minimum/midpoint/maximum vectors |
| Run mode / speed / current / acceleration writes | type 18; host/target routing; little-endian 16-bit index + two reserved zeros + four-byte value | mode 0x7005, current 0x7018, acceleration 0x7022, velocity 0x700A vectors |
| Operation status | type 2; host destination; motor/fault/mode in extended ID; big-endian position/velocity/torque/temperature | endpoints, model routing, fault and malformed-frame tests |
| Fault report | type 21 | transport fault latched; never treated as a joint sample |
| Physical mechanical zero | type 6 would change encoder coordinates | not implemented/commissioned; no hardware write exposed |

Fixed: SDK symmetric fields used 32767 scaling, reaching 65534 instead of the
specified full 65535. Both TX/RX now use the full span, with the usual half-step
zero quantization. RS03 scaling is pinned to its referenced manual. Home's
position path previously filled MIT velocity with an absolute speed limit,
which is actually velocity feedforward; it now sends zero feedforward. The core
still bounds Home targets at 0.1 rad/s. Generic position targets outside Home
must be trajectory-limited by their producer.

Only classic extended 29-bit, eight-byte status frames are decoded. CAN FD,
remote, invalid-DLC, foreign-host/motor and firmware-query responses are not
accepted as joint feedback. Non-finite MIT values reject without transmission.

## Lifecycle UI semantics

The four Enable/Disable controls call typed control_core operations, including
`joint_name=*` for all. Enable requires fresh complete feedback, released healthy
safety and no active operation. After enable admission the core emits bounded
zero-gain, zero-feedforward MIT keepalives until another command takes ownership,
Home/zero starts, Disable is requested, or a safety/feedback watchdog stops it.
This is not position holding. Enable/Disable report completion only after fresh
raw enabled-state feedback; a timeout submits Disable and reports unverified
physical state. Disable can be requested while offline or estopped, and can
interrupt an operation. Disabling during an exclusive Home/zero disables the
operation's affected joints. No estop reset or automatic re-enable is implied.

The firmware run-mode transition requirements and zero-coordinate reconciliation
are not proven by byte-level tests. The physical driver still needs commissioning,
including supported firmware, run-mode transitions while stopped, offsets,
polarity, linkage and real acknowledgement checks. No physical motor execution
is claimed by this audit.

## Verification

Protocol/adapter/management/watchdog tests live under
`humanoid_control/motor_control_hybrid/test/`; Web admission and batching tests
under `humanoid_control/humanoid_safety/test/`. SocketCAN verification uses only
an interface whose kernel type is vcan, reserved test IDs 241-243 and host 254.
Expected example, host 255/motor 1: Enable ID 0300FF01 and Disable ID 0400FF01,
both with payload 0000000000000000. Home is a sequence of type-1 targets, not a
single special home opcode. Sending a frame proves submission, not physical
execution or safety of gains.

Run the reproducible bounded SocketCAN check after sourcing ROS, rosenv and
install/setup.bash:

```bash
python3 scripts/check_can_wire.py --channel can1
```

It refuses physical CAN, emits only reserved virtual test IDs, verifies observed
frames against independent vectors, and writes `/tmp/can-wire-audit.jsonl`.
The fake-motor Web mode itself still does not emit CAN frames. The protocol
verifier exercises the real encoder/decoder and SocketCAN path separately.
