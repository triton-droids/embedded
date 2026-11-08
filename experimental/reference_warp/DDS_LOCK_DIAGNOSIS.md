# Native publication lock waits and scoped Cyclone DDS workaround

All runs here keep the gantry held and motors disabled (`start_chase:=false`).
They do not establish walking stability or real hardware timing guarantees.
The IMU source timestamps, reliable estop, 250 ms safety deadline, command expiry,
and explicit fault-reset behavior remain unchanged.

## Reproduction and native stacks

The system Fast DDS 2.14.5 build has `HAVE_STRICT_REALTIME=0`. In
`imu-nocamera-lock-1`, both viewer and camera rendering were disabled. The separate
observer received 2,999 IMUs with a maximum 19.69 ms receipt gap. Safety nonetheless
latched an IMU age of 340 ms while its received counter stopped advancing.
The `/safety/status` publication held the Python GIL while waiting 53.30 and
88.15 ms in native locks. These individual waits identify a contributing path;
they do not by themselves account for every millisecond of the 340 ms delay.

The actual stack is `rmw_publish -> DataWriterImpl::perform_create_new_change ->
DataWriterHistory::prepare_change -> remove_min_change -> change_removed_by_history
-> FlowControllerImpl::remove_change_impl -> pthread_mutex_lock`.
Fast DDS source `src/cpp/rtps/flowcontrol/FlowControllerImpl.hpp:1277–1321`
uses unbounded mutex acquisition in the non-strict build. With strict realtime
enabled, it attempts the corresponding locks until `max_blocking_time` instead.
Reducing KeepLast to 3 does not remove these locks or impose a time bound.

An experimental separate process for Safety diagnostics also failed:
`imu-nocamera-process-1` latched at 253 ms. In that run `/joint_states` waited
319.70 ms in the same history/flow-controller removal path, blocking the
simulator's Python GIL; the independent observer saw a 343.70 ms gap. The
separate-process feature was removed after the failed experiment. Its raw data
is retained, but that temporary launch flag is no longer supported.

Synchronous publication also failed (`imu-sync-held-2`): Safety latched at
254 ms and `/safety/status` waited 257.89 ms in `RTPSWriter::send_nts`.
Both simulator and Safety process environments were read back as SYNCHRONOUS.
`imu-sync-held-1` is **not a valid synchronous trial**: an inner launch still
overrode its requested mode to ASYNCHRONOUS. That override was corrected before
trial 2. Both raw runs are retained.

The lock interposer forwards every real call, return code and sample; it never
injects fresh IMUs. Slow-lock stack unwinding adds overhead, so these runs are
diagnostic evidence, not an overhead-free benchmark. It is never enabled by the
normal launcher. See `publish_probe/publish_boundary.cpp` and the retained
`results/*-trace/*.locks.txt` files.

Fast DDS documents that asynchronous publication may block when history fills,
and that synchronous sends execute in the caller's thread:
[official publication-mode explanation](https://discourse.ros.org/t/rmw-fast-dds-publication-mode-sync-vs-async-and-how-to-change-it/17153).
The native stacks above, rather than the mode names, establish where this
installation actually waited.

## Runnable workaround

`run_chase.sh` activates the workspace plus privately extracted Cyclone DDS
packages, selects `rmw_cyclonedds_cpp`, and starts the same chase launch. System
ROS packages and generic launch defaults are not overwritten. No sudo is used.

```bash
./experimental/reference_warp/run_chase.sh start_chase:=false use_viewer:=false localhost_only:=0
```

Camera/depth rendering stays on with `use_viewer:=false`. Omit that argument for
the MuJoCo GUI. The Web monitor remains on port 8182. A fresh checkout needs
`bash experimental/reference_warp/prepare_cyclone_trial.sh` once to download and
extract the private middleware packages. A fresh shell with the original direct
ROS launch remains available for Fast DDS diagnosis.

## Limits

Three new full-camera runs used the system ROS libraries plus privately extracted
Cyclone DDS. `/proc/<pid>/maps` confirmed Cyclone in all 11 ROS processes, with
no Fast DDS library mapped. No native timing shim was loaded in these trials.

| Run | Requested duration | IMU samples | Max receipt gap | IMU estop | WBC outcome |
| --- | ---: | ---: | ---: | --- | --- |
| `imu-cyclone-camera-gui-1` | 32 s | 2,892 | 21.76 ms | none | compute deadline, 1,599 solves |
| `imu-cyclone-camera-gui-2` | 32 s | 2,880 | 43.49 ms | none | no latched WBC fault, 2,662 solves |
| `imu-cyclone-camera-web-90s` | 90 s | 8,662 | 58.01 ms | none | expired input deadline, 1,732 solves |

All three enabled the MuJoCo GUI and RGB-D rendering. The third also polled
the read-only Web `/api/status` endpoint at 5 Hz: 430 valid JSON responses and
21 failed requests were counted, including startup before the server existed.
IMU observation exposure in that run was 86.91 s after startup, not 90 s of
post-release walking. Physics stayed unpaused and the gantry stayed held.
The WBC expired-input fault occurred with only 1.481 ms computation and 0.216 ms
solver time: this differs from the first trial's computation overrun. No deadline
was extended to hide either failure. Some nodes emitted exceptions during the
bounded SIGINT shutdown; these are retained in raw logs and are not runtime
IMU faults.

`imu-cyclone-source-loss` then deliberately SIGSTOPed the held simulator for
1.01 s. Safety latched `IMU age=0.252s limit=0.250s`; the source resumed and IMU
reception recovered, while the safety latch remained asserted through the end
of the 22 s run. This checks actual DDS source loss and recovery, not a mocked
freshness calculation. The source-pause option refuses `start_chase:=true`.

Validation: 20 targeted transport/safety tests passed (one separately gated DDS
integration test skipped in that batch). Shell syntax, the new wrapper's real
`--show-args` invocation, and `git diff --check` passed. Results and all failed
attempts remain under `results/`.
The IMU time-contract suite was then rerun against Cyclone DDS with
`RUN_ROS_INTEGRATION=1` on isolated domain 229: all 16 tests passed, including
paused ROS clock/replayed IMU rejection over real DDS.

Cyclone DDS is a measured workaround for the observed IMU blocking; it does not
repair Fast DDS or guarantee all control deadlines. The first full-camera GUI
trial hit a separate WBC computation deadline after 1,599 solves (22.659 ms,
solver 2.964 ms). Keep this failure in the denominator. Do not report a healthy
IMU as proof that WBC or chasing is healthy. Turning remains unimplemented.
