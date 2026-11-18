# Command and joint-state watchdogs

The CAN backend owns an independent `command_timeout_s` (default 0.5 seconds,
finite and strictly positive). The hybrid launch exposes it as
`can_command_timeout_s`; it is separate from the C++ upstream `cmd_timeout_s`.
The 50 Hz control configuration is unchanged.

## CAN command lifetime and recovery

- Every non-disable command must carry a nonzero ROS header timestamp. Reject
  commands older than the timeout or more than 10 ms in the future. Publishers
  on other computers must use synchronized ROS clocks.
- Convert the remaining source lifetime to a host monotonic deadline on receipt.
  Queue residence and host scheduling consume that lifetime. Updating
  one joint never renews another joint's deadline.
- Motors start inhibited. An explicit fresh enable is required before motion.
  An enable also arms the watchdog, even before the first motion command.
- Expiry removes the active command and sends disable to that joint. Failed
  disables are retried. Subsequent motion cannot recover it: an explicit enable
  received after expiry and fresh motion are required. A queued pre-expiry
  enable cannot rearm it. Disable remains accepted without a timestamp.
- Source disconnect, process death upstream, or a paused ROS clock cannot keep
  an active command alive indefinitely. Timeout is checked on worker progress,
  not by a hard real-time interrupt. Motor-feedback waits are no longer in TX.

## Asynchronous CAN transport

The repository-owned `can_transport.py` reuses SDK MIT encoding without editing
the installed SDK. Each interface has one independent RX thread. The TX worker
submits nonblocking SocketCAN writes (`timeout=0`, nonblocking socket), including
enable, disable and parameter writes; it never receives acknowledgements.
Enable completion is asynchronous: targets stay inhibited until RX observes a
fresh, fault-free type-2 Motor/run status for that exact joint after its enable
submission. Motion traffic cannot extend an unacknowledged enable's deadline.
Mode 2 in that status is the Motor/run state, not the velocity-control parameter
value. This follows the [RobStride type-2 status definition](https://www.robstride.com/assets/product_manual_robStride02-e7f9f7c4.pdf).
RX has a bounded 10 ms idle wait that returns immediately when data arrives;
that wait does not delay TX. Frames are dispatched by actual motor CAN ID and
host ID before model-specific decoding. Fault/error frames latch a local stop
that a supervisor `false` message cannot reset; restart after diagnosis.

Fresh whole-message target batches wake TX immediately. A bounded mailbox holds
at most two entries per joint (latest lifecycle request and latest target), so
old targets are coalesced instead of replayed. A pending disable drops motion
until a new enable. Hold-last retransmission remains nominally 50 Hz, using only
the remainder of a monotonic period; there is no extra 20 ms sleep after work and
no 50 ms per-motor enable delay. Shutdown joins both owners before closing CAN.

Successful writes prove only host submission, not physical execution or stop.
Send failures are counted and retry on later worker passes while commands remain
valid. `/can_timing` exposes per-interface submission/error counts, worker cycles,
maximum work duration, RX counts and mailbox size. Worker cycles are not motor
update rate. Actual arbitration/adapter/motor latency still requires measurement.
The old `feedback_poll_hz` and `feedback_poll_when_idle` parameters are accepted
for launch compatibility only; they no longer schedule synchronous reads.
RX is passive and continuously consumes the motor's status reports. Disabled
motors that do not emit status cannot be made fresh by reading cached values;
their feedback-report behavior must be commissioned separately. No automatic
enable, protocol switch, active-report reconfiguration or zeroing is performed.

## Joint feedback validity

Torch and C++ keep separate receipt and source timestamps for every joint.
Position and velocity must both be finite and present; duplicate names or a
malformed frame invalidate cached validity. Partial valid messages may update
individual joints, but never renew missing joints. Every required joint must
pass both receipt-age and source-age checks. Unstamped and future frames fail.
Torch's default joint timeout remains 0.25 seconds; C++ remains 0.2 seconds.
These state timeouts must be finite and positive.

The CAN publisher uses the oldest included host-receipt time as its single
JointState timestamp. This conservatively preserves cached sample age; it is
still not a synchronized motor hardware capture timestamp.

Both launch files pass the full expected joint order into C++ as `joint_names`.
The hybrid launch obtains it from the selected motor configuration. Direct
invocations should also set `joint_names`; the compatibility fallback infers
the first valid message's names and cannot detect joints never observed.
C++ disables all expected joints when any required sample fails, and discards
cached upstream commands/hold-last state so feedback recovery cannot replay a
pre-fault enable. Torch stops publishing while its required feedback is invalid.

## Software verification

`test/test_control_watchdogs.py` calls production Python callbacks with mock
transport and deterministic clocks, without starting hardware or loading a
policy. `test/test_cpp_state_watchdog.cpp` exercises the real C++ callbacks and
the control-loop fault transition in a separate ROS domain. Build with colcon;
run the registered `control_watchdogs_test` and `cpp_state_watchdog_test` tests.
Use the existing ROS/Python environment with the CAN runtime dependencies.

These tests verify software state transitions. Actual bus disable latency,
single-motor disconnect behavior and safe mechanical stopping remain hardware
commissioning tests. The installed SDK is not modified.

## Asynchronous ONNX inference and end-to-end expiry

The tracking policy now has exactly one inference worker. The ROS executor
continues receiving IMU, joints, estop and status requests during inference.
There is no pending inference FIFO: each scheduled 50 Hz iteration snapshots the
latest immutable IMU/joint tuples and previous accepted raw action. Missed periods
are skipped; no catch-up inference burst is generated. Reference/action semantics,
model PD, angle bounds, polarity and ankle conversion remain unchanged.

Both inputs must pass receipt/source age checks (100 ms by default). In real
joint feedback mode their source stamps must differ by at most
`max_input_skew_s` (default 20 ms), otherwise inference waits for aligned inputs.
This only bounds reported timestamp skew; individual joint sample edges are not
hardware-synchronized. Waiting produces no fresh commands, so downstream expiry
still stops a previously active target.

After inference, validate the **original snapshot** again, check estop/fault/
shutdown and finite output, and enforce `max_inference_s` (default 20 ms, at most
one trained control period). An over-budget/expired result is discarded and the
fault latches until node restart; rejected results do not advance action history.
This conservative budget is a deployment threshold, not measured Jetson capacity.

`MotorCommand.header.stamp` carries the oldest snapshot source stamp.
`valid_until` carries that stamp plus the input timeout. C++ preserves the oldest
held command source and earliest expiry, and keeps a separate monotonic deadline
for each held joint. CAN takes the earlier of its normal watchdog deadline and
explicit expiry, translating the remaining lifetime to monotonic time. Neither
forwarding nor retransmission renews command lifetime. DISABLE remains accepted
without a valid timestamp. Zero `valid_until` preserves legacy source-time
watchdog behavior; it does not disable the watchdog.

The message interface changed: rebuild `motor_control_interfaces` and every
consumer on **all participating hosts**, then restart all nodes. Mixed old/new
DDS message definitions are unsupported. Re-source the rebuilt workspace.

`inference_deadlines_test` uses a fake policy with controlled clocks and a blocked
worker to verify callback progress, estop, stale-snapshot rejection, source-stamp/
expiry preservation, history and shutdown gating without ONNX or motor hardware.
C++ and CAN regression tests cover explicit expiry and per-joint hold deadlines.

Policy stream publishers and the C++ desired-MotorCommand subscription use
best-effort depth 1 QoS to avoid reliable-delivery backlog in this path. Legacy
reliable publishers remain compatible with the C++ best-effort subscription;
subscribers consuming policy monitor topics must permit best-effort delivery.
The IMU gravity filter integrates using source read times rather than host
callback arrival intervals, and duplicate/reordered IMU stamps are rejected.

For the software-only DDS relay smoke test, after sourcing ROS and the rebuilt
workspace, run from the workspace root:

```bash
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/deadline-probe-logs \
  python3 humanoid_control/motor_control_hybrid/test/ros_deadline_relay_smoke.py
```

It starts only an isolated C++ relay with fake a/b feedback and private topics.
It verifies actual forwarded source/expiry fields and deadline-triggered disable
while feedback remains fresh. No CAN/backend/enable node is started. DDS requires
local sockets; a sandbox socket-denial is not evidence of a control failure.

## Advisory supervisor / SafetyFeature warnings

The C++ control node independently subscribes to `/safety/status` and looks for
`humanoid_safety/supervisor`. This is advisory only: loss of this heartbeat or an
unhealthy diagnostic level does not stop, freeze, or replace motor commands.
Explicit `/safety/estop`, state freshness and command/CAN expiry still retain
their existing disable behavior. The external IMU integration is unchanged.

- `supervisor_warning_timeout_s`: 0.5 s, finite and positive.
- `supervisor_warning_startup_grace_s`: 3 s, finite and nonnegative.
- A separate wall timer checks once per second. Warning logs are throttled to
  once per five seconds using a steady clock; this is not a real-time deadline.
- Source-stamp and monotonic receipt age both count. Zero/future/expired stamps,
  unrelated diagnostic entries and replayed stamps cannot renew the heartbeat.
- A fresh OK heartbeat clears the condition. The monitor does not publish an
  estop or reinterpret an existing asserted estop as a warning.

Production code currently registers **zero `control_core::SafetyFeature`
plugins**. The interface remains available, but this change adds no physical
safety algorithm or arbitrary hardware threshold. If a plugin is added later,
`check=false` or a thrown exception populates `Output.warnings`; normal current
command selection continues, rather than returning the previous target early.
The ROS wrapper emits those messages as throttled warnings. Python bindings
expose the same `warnings` vector. Test-only failing plugins verify this contract.

Rebuild `control_core` and `motor_control_hybrid` together after the C++ API
change, and restart consumers using the rebuilt libraries/Python bindings.
The regression suite verifies missing/expired/unhealthy supervisor diagnostics
remain advisory, explicit estop still disables, and failed/throwing test plugins
cannot freeze the new target.
