# RobStride abstract commands: manual review and interface design

Status: researched design with initial backend service scaffolding; zero and
Home execution are not implemented. Checked on
2026-10-21. No motor was queried, enabled, moved, zeroed, or flashed during this
review. The existing `hardware_verified: false` guard remains unchanged.

## Official manual baseline

The selected motor registry maps IDs 1/4/6/9 to RS04, 2/3/7/8 to RS03, and 5/10
to RS02. This is repository configuration, not physical identity evidence.

The official [Product_Information repository](https://github.com/RobStride/Product_Information)
was inspected through its recursive tree at commit
`384cc561e1ff079c22f68b19908526094430febd`. Its README links still omit the new
`CAN2.0` directory and do not reliably identify the newest manuals. The files
actually present in the tree were used instead. English and Chinese revisions
are not necessarily synchronized.

| Configured model | Latest Chinese CAN2.0 manual found | Mechanical-zero table | Zero-calibration rules | Parameter table |
|---|---|---|---|---|
| RS02 | `RS02使用说明书261009.pdf` | PDF/printed p.34 | Section 4.2.6, PDF/printed p.40 | PDF/printed p.38 |
| RS03 | `RS03使用说明书261013.pdf` | PDF p.34 / printed p.33 | Section 4.2.6, PDF p.39 / printed p.38 | PDF p.37 / printed p.36 |
| RS04 | `RS04使用说明书261009.pdf` | PDF p.36 / printed p.35 | Section 4.2.6, PDF p.42 / printed p.41 | PDF p.40 / printed p.39 |

RS03 was also cross-checked against `RS03User Manual261013.pdf`: mechanical
zero at PDF p.44 / printed p.43, and section 4.2.6 at PDF p.53 / printed p.52.
The latest English RS02/RS04 manuals in the inspected tree still carry date
260713, so the newer Chinese manuals are the baseline for those models.

Permanent source URLs, file sizes, and SHA-256 hashes are recorded in
[robstride_manual_sources.json](robstride_manual_sources.json). These dates
identify document files; they do not prove the installed motor firmware.

## Verified CAN command mapping

Scope: the manuals' CAN2.0 extended-frame protocol used by the existing backend.
Do not mix its operation numbers with the separate standard-frame or CANopen
chapters. The extended identifier is assembled as:

```text
identifier = (communication_type << 24) | (extra_data << 8) | destination_id
```

For host-to-motor lifecycle requests below, `extra_data` contains the host ID.
These are protocol descriptions, not instructions to transmit on a live bus.

| Abstract intent | Protocol mapping | Required completion evidence |
|---|---|---|
| `DISABLE` / stop | Type 4, eight zero data bytes. Byte 0 = 1 is a different operation that clears faults; never substitute it for an ordinary stop. | Fresh feedback reporting a non-running state; a successful socket send is insufficient. |
| `ENABLE` | Type 3, eight zero data bytes. | Fresh type-2 status reporting Motor mode, no fault, and the approved execution conditions. |
| `SET_MECHANICAL_ZERO` | Type 6, payload `01 00 00 00 00 00 00 00`. Sets the present motor mechanical coordinate to zero. | Post-command motor-coordinate readback and stopped-state checks; generic feedback arrival alone is not proof. |
| Read `run_mode` / mechanical position | Type 17, index in bytes 0-1, little-endian; reserved/request bytes zero. `run_mode=0x7005`, `mechPos=0x7019`. | Matching type-17 reply from the selected host/bus/motor and requested index. |
| Configure execution mode | Type 18, `run_mode` index `0x7005`; mode value encoded as uint8. Chinese tables specify 0=motion, 1=PP, 2=velocity, 3=current, 5=CSP. | Stop first, then matching parameter readback. Never change mode implicitly to enable a UI button. |
| `HOME` | No single home packet in this reviewed extended-frame interface. Generate bounded trajectory targets, then use the approved motion-mode backend. | Actual tracking and final settling checks, not elapsed trajectory time alone. |
| Firmware identification | The Chinese manuals show type 4 with `00 C4 00 00 00 00 00 00`; reply is type 2 with prefix `00 C4 56` and version bytes 3-6. | Dedicated management-response decoding; do not publish these bytes as angles. Treat identification as a managed operation rather than assume it is a harmless query during motion. |

`zero_sta` at `0x7029` selects the power-on coordinate range. `add_offset` at
`0x702B` changes a coordinate offset. Neither is interchangeable with the type-6
mechanical-zero operation or a home movement.

## Firmware and mode restrictions

All three reviewed Chinese manuals state that zero calibration is available
in CSP and motion control, and blocked in PP. Their section 4.2.6 distinguishes
old firmware, which can chase an old target after zeroing, from newer firmware,
which updates the target to zero in the supported modes. The section does not
supply a universal minimum firmware build proving the behavior for all models.

Consequently, an implementation must use a verified model/firmware/protocol
profile. A supported model name or a newer document date alone cannot authorize
zeroing. Unknown firmware, unknown execution mode, or PP must reject the request.
Stopped-state verification, single-motor selection, static feedback, and a
mechanically supported setup are additional application requirements; they do
not establish that every firmware version is safe to zero while stopped.

The current motor-coordinate/robot-joint conversion also matters. Hardware
zeroing changes the encoder coordinate system. Software offsets, joint limits,
policy input history, and nonlinear ankle calibration cannot be silently reused
without checking their new meaning. Do not zero a motor and keep sending cached
pre-calibration targets, or apply a second software offset under an ambiguous
"Set Zero" operation.

The manuals list `limit_torque` at `0x700B`, `limit_cur` at `0x7018`, and a CAN
watchdog at `0x7028`. That does not prove that any one parameter caps total MIT
PD output on every firmware. The RS02 manual itself has different ranges in
its parameter table versus its motion/feedback tables (including 14 versus
17 Nm, and 33 versus 44 rad/s). Resolve model/firmware behavior before writing
limits or changing codec tables. Do not copy these example ranges into safe
robot operating limits.

No automatic data-save, protocol switch, fault clear, firmware update, or
unconditional re-enable is part of `SET_MECHANICAL_ZERO`. Controlled restoration
of previously enabled control is allowed only after the verification and
coordinate-rebinding steps below. Persistence and readback after a power cycle
require separate validation.

## Abstract command ownership

Keep `Interfaces/control_core` independent of ROS. The existing
`cpp_control_node` is its ROS adapter: it receives commands, supplies validated
state to the library, and owns downstream ROS clients. The library owns
operation admission, exclusion of normal motion, and progress/state decisions.
The CAN backend remains the only owner of CAN TX/RX.

```text
Web UI / CLI
    | service client: /control_core/submit_abstract_command
    | or command publisher: /control_core/abstract_command
    v
cpp_control_node (service server + topic subscriber)
    | both ingress paths feed the same bounded admission queue
    v
control_core operation state machine, ticked at 50 Hz
    | approved backend operation / normal command output
    v
cpp_control_node (client: /motor_backend/execute_operation)
    v
python_can_node (service server, queues work to existing TX owner)
    | CAN response -> existing RX owner -> typed result/state
    v
/control_core/abstract_command_status -> Web UI
```

The names above are proposed interfaces, not endpoints that already exist.
A topic and a service are different ROS interfaces. The topic supports the
requested abstract-command ingress; the service gives UI clients an immediate
admission result. Both must use identical validation and deduplication. A service
response saying "accepted" never means a motor action succeeded.

### Proposed request and status contract

A typed `AbstractCommand` request should include:

- `request_id`: unique operation identifier, mandatory for all state-changing
  requests; shared across topic/service ingress for deduplication.
- Source stamp and expiry: reject stale, future, or already expired commands.
- Operation enum: `HOME`, `SET_MECHANICAL_ZERO`, `ENABLE`, `DISABLE`, `CANCEL`.
- Explicit selected joint names; mechanical zero is single-motor only in the
  first implementation. Never treat an empty list as "all motors".
- Calibration/profile revision: bind the request to the verified coordinate and
  firmware configuration, rejecting stale UI state.
- `cancel_request_id` for cancellation. A home profile selects server-validated
  speed, acceleration, tracking-error, and torque/current limits; clients cannot
  bypass limits by submitting arbitrary raw CAN bytes.

`SubmitAbstractCommand.srv` should contain that request and return
`accepted`, `request_id`, and a reason code/message. Long operations run
asynchronously. Publish status states such as `REJECTED`, `ACCEPTED`,
`DECELERATING`, `WAITING_FOR_STOP`, `RUNNING`, `VERIFYING`, `REBINDING`,
`RESUMING`, `SUCCEEDED`, `FAILED`, `CANCELED`, and `UNKNOWN_OUTCOME`.
Include the selected joints, progress, fresh telemetry,
and failure reason. A future ROS action can expose the same lifecycle directly;
it must not introduce a second execution path.

Command topics must be reliable and volatile: never replay a latched calibration
request to a restarted controller. Bound the queue, reject concurrent conflicting
operations, and retain a deduplication record. After a process restart or an
uncertain backend result, do not automatically replay a state-changing request.
A timeout after transmission is `UNKNOWN_OUTCOME`, not a reason to retry zeroing.

## Mechanical-zero transaction

The requested behavior is to pause normal execution, decelerate motion under
core ownership, set the selected motor's mechanical zero, verify it, and then
restore control in the new coordinate system. This is a managed transaction,
not a direct zero write from the UI.

```text
ACCEPTED -> DECELERATING -> SETTLING -> WAITING_FOR_STOP
         -> ZERO_WRITE -> VERIFYING -> REBINDING -> RESUMING -> READY
```

Failures enter a latched failure/unknown-outcome state; they do not automatically
resume. The sequence runs only after the actual firmware and execution profile
are verified. In particular, it does not bypass the current hardware guard.

### Deceleration and pause

- Pause all normal motion owned by the core, even when only one motor will be
  zeroed. Acquire exclusive backend ownership and reject competing policy,
  manual, SDK, and WebSocket motion. Save the prior command-owner identity and
  enable mask, not a queue of commands to replay later.
- Read fresh actual positions and velocities. Begin the stopping trajectory
  from those states and enforce validated deceleration, acceleration/jerk,
  joint-range, path, and actual motor-output limits. A fixed-duration ramp or
  multiplying incoming targets by a decreasing scalar is not sufficient.
- Do not immediately send type 4 for this normal controlled-stop request: type
  4 disables execution rather than generating a gradual stopping trajectory.
  Continue the bounded stop targets until measured velocity is sufficiently
  small and the position has settled for the configured dwell interval.
- A stop timeout, stale feedback, estop, drive fault, exceeded envelope, or
  unexpected movement takes the existing fault/stop path. Gradual deceleration
  must never delay an emergency response. The software must not claim a safe
  supported stop when the required physical/output constraints are unknown.
- The independent head MCU currently is not owned by this core. A body-control
  lease must not be reported as an all-robot pause; head control needs explicit
  participation before it can be included in that scope.

### Zero write and confirmation

The transaction, after a profile is verified, is:

1. Admit a single selected motor with known bus/ID/model/firmware and mapping.
   Verify `hardware_verified`, fresh raw feedback, permitted mode, a validated
   stopping envelope, and an explicitly prepared supported setup. Unknown
   conditions reject. Initial motion is allowed only if it can be stopped by
   the controlled deceleration phase above.
2. Acquire an exclusive backend operation lease for normal core-controlled
   motion, discard queued normal targets/pending enables, and complete the
   deceleration and settling phase. Use transaction-owned stop targets whose
   timestamps/expiry are generated by that operation, not renewed old targets.
3. After settling, request the required stopped/disabled state through the sole
   CAN TX owner and verify it from fresh feedback. This stop is separate from
   the preceding deceleration. Do not clear faults or the supervisor estop.
4. Revalidate the operation/profile after stopping. Only the verified profile may
   authorize a type-6 write. No all-motor calibration loop or automatic retry.
5. Observe post-write feedback and issue serialized readback as required. Keep
   mechanical-zero readback separate from mapped joint coordinates. Type-2
   arrival does not carry a request ID, so unrelated periodic traffic must not
   be mistaken for an acknowledgement.
6. Invalidate coordinate-dependent caches/history and review affected offsets,
   limits, and ankle mapping. Mark calibration-dependent readiness unverified
   until the new mapping is validated. Persist an audit result, not an automatic
   `hardware_verified: true` change.
7. Restore control only through the coordinated resume phase below. Without
   adequate evidence, retain failure/unknown outcome and do not re-enable.

### Coordinate rebinding and resume

Resume means returning control to an authorized command owner after a verified
transition. It never means replaying pre-zero targets in a changed coordinate
system. The zero-write response and the resume result are separate milestones.

1. Invalidate held motion, pending enables, trajectory references, cached
   feedback, policy histories, and commands bearing the previous calibration
   revision. Drain old input streams while the core owns the transaction.
2. Establish fresh actual state in the new coordinates. Reconcile software
   offsets, limits, home targets, and nonlinear ankle mapping. Increment the
   calibration revision only after validating the new mapping. Unknown or
   inconsistent mapping stops the transaction at this stage.
3. Initialize the new hold reference from measured post-zero position with zero
   target velocity, using the verified backend's safe enable/target ordering.
   Publishing a target before enable is not proof that the motor accepted it;
   the current CAN backend drops motion before enable acknowledgement.
4. If normal control was enabled before the operation, restore it only through
   the verified enable sequence with fresh acknowledgement and matching
   coordinate revision. Previously disabled motors remain disabled. Revalidate
   estop and feedback throughout; never clear a safety latch to resume.
5. Release the lease and accept only fresh, revision-matching commands from the
   resumed owner. Policy control must restart/reinitialize its observation and
   reference contract rather than blindly resume an old action vector. An old
   trajectory may continue only if it has been explicitly transformed and
   validated against the new mapping and current state.

If the zero operation succeeds but resume cannot be verified, report
`ZERO_VERIFIED_RESUME_BLOCKED` with the reason. Do not collapse this into a
generic success or retry zeroing. The first hardware implementation must keep
resume unavailable until the relevant backend/profile capabilities exist.

Cancellation after a zero write cannot undo the changed coordinate. Report that
fact rather than claiming a rollback. A stop request does not establish that an
unsupported firmware profile can safely calibrate.

## Gaps in the current implementation

- `Controller::step()` retains ordinary subset/hold behavior. A separate pure
  `ZeroOperation` now owns the stop/wait/failure transaction; its ROS integration
  is described in the implementation checkpoint below. Home remains unimplemented.
- `cpp_control_node` now has abstract-command topic/service ingress, status output
  and asynchronous backend clients. Real zero admission remains disabled by default.
- The CAN backend exposes `motor_backend/get_feedback` (`GetMotorFeedback`) and
  `motor_backend/execute_operation` (`ExecuteMotorOperation`), relative to its ROS
  namespace. The first returns cached raw type-2 feedback without transmitting
  CAN. The second recognizes the typed `SET_MECHANICAL_ZERO` operation but rejects
  it as not ready: no exclusive calibration lease, verified firmware/mode profile
  or zero readback transaction exists yet. Its mailbox still handles only
  enable/disable and motion. No service callback sends type 6.
- RX drops type-2 firmware-shaped replies with prefix `00 C4 56` instead of
  interpreting them as joint angles or enable acknowledgements. Full correlated
  management-response decoding is still required before firmware queries.
- The socket filter currently admits type 2 and fault reports, not type-17
  parameter replies. Parameter request/response correlation is not implemented.
- Ordinary telemetry omits durable typed enabled/run-mode/firmware state needed
  for calibration admission. These must not be inferred from a UI button click.
- `MotorSettings.command()` clips feedforward torque only; it does not establish
  a total output-torque bound for the embedded motor PD loop.
- The current Web UI intentionally exposes GET-only telemetry. A future control
  client must submit requests through the core, never publish raw motor commands.

## Implementation and verification order

1. Add typed management decoding and model/firmware profiles with canned CAN
   fixtures; no hardware transmission. Cover version replies, type-17 responses,
   unknown profiles, faults, source identity, and late/out-of-order messages.
2. Add the abstract-command data contract and pure-core transaction state
   machine. Test duplicate requests, timeout/unknown outcome, cancellation,
   stale feedback, estop, and exclusion of normal motion. Cover stopping from
   nonzero velocity, settling timeout, post-zero coordinate revision, blocked
   resume, and refusal to replay old targets or enable previously disabled motors.
3. Connect the ROS adapter's topic/service ingress and downstream asynchronous
   client to a fake management backend. Test admission separately from completion
   and ensure no callback blocks the 50 Hz control loop.
4. Add Web UI controls that display admission, progress, and verified completion.
   Home is unavailable until its path and actual output limits are verified.
5. Validate the exact physical motor and firmware, one mechanically supported
   motor at a time, through a separate explicitly authorized commissioning step.

This review deliberately adds no executable calibration or motion path. Reading
the newest documentation resolves packet semantics; it does not supply the
missing installed-firmware identity or prove the robot's mechanical readiness.

## Initial backend services

| Service (default namespace) | Request | Response semantics |
|---|---|---|
| `/motor_backend/get_feedback` | `GetMotorFeedback`: one `joint_name` | Cached raw motor rad, rad/s, Nm, temperature, status mode, fault bits, source stamp and age. `available=false` means no sample; missing numeric fields are NaN, not measured zero. `fresh=false` retains a stale sample for diagnosis. `success` additionally requires no reported or latched transport fault. It does not establish zero calibration or motor enable. |
| `/motor_backend/execute_operation` | `ExecuteMotorOperation`: `request_id`, `joint_name`, typed `SET_MECHANICAL_ZERO` constant | Always `accepted=false`, `state=REJECTED` at this stage, with a reason. Neither an HTTP nor ROS client can bypass the missing transaction by supplying a raw CAN type 6 or a caller-asserted stopped flag. |

ROS operation enum values are separate from CAN communication-type numbers.
CAN type 2 is received status, not a command to request home. CAN type 6 sets
the mechanical zero; receipt of a later type-2 packet cannot alone prove that
zeroing succeeded. Status mode 2 (Motor/run) is also distinct from run_mode 2
(velocity control).

Home admission belongs in the future control_core command service/topic. Its
trajectory is generated by the core and streamed via the existing
`/motor_commands` publisher to the CAN backend. A frontend must not publish
home targets directly to bypass the core's pause/ownership rules. None of these
services automatically enables, homes or zeros a motor at startup.

## Core implementation checkpoint

The pure C++ `control_core::ZeroOperation` now implements measured-state
admission, bounded commanded deceleration of all core-owned joints, continuous
measured settling, one-shot backend dispatch, separate acceptance/rebind phases,
request-ID replay prevention, timeout/cancel/failure latching and an explicit
reset. Its limits bound requested motion and trip on measured effort; they do
not establish a physical total-torque bound or prevent sub-cycle damage.
Previously disabled joints receive DISABLE throughout. After backend submission,
old-coordinate hold targets are suppressed. An uncertain zero outcome cannot be
cleared with RESET. A definitive rejection permits explicit RESET, which still
discards all cached commands and requires a fresh enable/command sequence.

The ROS C++ adapter exposes `/control_core/abstract_command`
(`AbstractCommand`), `/control_core/submit_abstract_command`
(`SubmitAbstractCommand`) and `/control_core/abstract_command_status`
(`AbstractCommandStatus`). Both ingress paths share admission. Requests require
fresh header/expiry times and IDs; CANCEL/RESET reference the current request ID.
The adapter asynchronously reads typed raw status for actual enabled-state
admission and calls the existing Python backend operation service after settling.
No service callback waits for CAN or a future. Ordinary targets received while
paused are dropped. Existing estop/state watchdogs preempt the operation.

`abstract_actions_enabled` defaults to false and the unified launch leaves it
false. The real backend still rejects zero: an exclusive backend lease,
commissioned firmware/run-mode profile, raw-zero verification, coordinate mapping
rebind, producer-history reset and verified lifecycle recovery are not implemented.
The pure core's `confirm_rebind` hook is tested with fixtures only; the ROS adapter
exposes no endpoint that can manufacture that confirmation. A backend ACCEPTED
response alone remains in WAITING_REBIND and ultimately times out disabled.
Home trajectory generation and its admission interface also remain unimplemented.

The unified `motor_control_hybrid/control_test.launch.py` combines CAN (or
explicit fake motors), C++ core, one safety supervisor, Web telemetry, the existing
safety-owned joint relay and model TF/RViz. It generates no startup motion.
