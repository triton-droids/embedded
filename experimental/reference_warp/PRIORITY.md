# IMU > WBC > other experimental scheduling

Implemented and measured on 2026-10-31. This profile is enabled by default only
in `experimental/reference_warp/chase.launch.py`. Generic launch defaults and
all control/safety deadlines are unchanged. No hardware was operated.

## Actual hierarchy

The current user has no effective capabilities, `RLIMIT_RTPRIO=0`, and cannot
use passwordless sudo. `priority_mode:=auto` therefore resolves to **nice** on
this host, with `SCHED_OTHER`, not real-time FIFO. Lower nice means higher
relative scheduling weight. Startup prints the resolved profile and each
configured thread's actual policy, priority, nice, affinity, PID and TID.

| Execution role | Nice | This host's logical CPUs | Optional FIFO priority |
|---|---:|---|---:|
| Body IMU executor, physical sample producer, Safety receive/watchdog | 0 | 0, 4 | 70 |
| WBC process | 5 | 1, 5 | 60 |
| Policy, C++, camera, viewer, ordinary simulator executors, diagnostics, Web and tracking | 10 | 2, 3, 6, 7 | SCHED_OTHER |

Affinity groups are discovered from the allowed CPUs' physical package/core
topology; SMT siblings stay together. On this four-core workstation the first
physical core is assigned to the IMU chain, the second to WBC and the remaining
two to other work. Allocations with fewer than three physical cores share the
allowed CPUs rather than inventing an empty group. These are affinity partitions
of this stack, **not exclusive kernel CPU isolation**: unrelated applications,
kernel work and interrupts have not been moved.

The simulator bootstrap must stay at nice 0 until critical threads are created;
it then demotes its main thread. Otherwise unprivileged children could not
restore nice 0. Camera and viewer workers set their own ordinary role. Native
DDS/library threads within the simulator can be shared dependencies of IMU
publication; they are not individually classified or claimed to have unique
per-topic priorities. Separate Python executors also still share the GIL and
the simulator's physics lock. This is not complete process/memory isolation.

## Diagnostics isolation

When the profile is enabled, Safety creates a separate diagnostics node and
single-thread executor for `/safety/status`. Its receive/watchdog/reset executor
retains the IMU role. Trip/reset handling no longer performs synchronous
diagnostic publication on the critical executor; the diagnostics timer reports
the updated state. Reliable/transient-local `/safety/estop` is still published
immediately from the critical path before fault logging. Other enabled safety
observations remain serialized with the supervisor; state checks were not
removed or evaluated concurrently.

For dedicated simulated IMU execution, `/imu/status` runs on an ordinary
diagnostics node; the high-priority body IMU executor handles sensor sampling
and publication. The physical sample producer is in the IMU tier because sensor
priority cannot create new samples if physics stops. Source stamps still come
from actual physical acquisition; stopped physics cannot be made fresh by
republishing old data.

The Safety diagnostics change is opt-in through the experimental scheduler
environment. Normal Safety construction/launch retains its original behavior.
The simulator's ordinary default remains without a dedicated IMU executor.
These changes do not rewrite the controller in C++ or guarantee bounded DDS
publish/GIL/lock latency.

## Launch

```bash
source scripts/activate_rosenv.sh
MUJOCO_GL=egl ros2 launch "$PWD/experimental/reference_warp/chase.launch.py" \
  start_chase:=true use_viewer:=false localhost_only:=0 priority_mode:=auto
```

Modes:

- `auto`: choose FIFO if the launching process has `CAP_SYS_NICE` or an RT
  priority limit of at least 70; otherwise explicitly report and use nice.
- `nice`: use the tested 0/5/10 hierarchy and affinity partition.
- `fifo`: require permission for FIFO 70/60; abort launch if unavailable.
- `off`: disable the scheduling hooks, affinity partition and Safety diagnostics
  split. Dedicated simulated IMU diagnostics remain separate when the dedicated
  IMU executor is configured.

Start the launch at nice 0. The wrapper never invokes sudo, changes system
configuration or installs capabilities. FIFO eligibility is checked before
launching nodes, and each configured execution thread verifies actual readback;
runtime configuration errors are reported rather than labeled successful.
**FIFO execution has not been validated on this unprivileged host.** Merely
enabling it on another host does not establish hard real-time performance.

## Verification and retained outcomes

The simulator and Safety packages built successfully. The selected simulator,
Safety time-contract/status, reference-warp/chase/payload and priority suites
completed with **134 passed, 1 skipped** on the final source. The included
diagnostic-isolation differential suite also separately completed **3 passed**:

- Actual Linux child threads read back nice 0/5/10 and their expected affinity.
- Under a blocked Safety diagnostic publication, the original shared executor
  processes no new IMU callbacks during the observation; the separated executor
  accepts at least ten fresh samples. Actual IMU loss still latches the unchanged
  250 ms watchdog while diagnostics remain blocked.
- A blocked simulated-IMU status callback does not prevent at least five new
  IMU publications within 250 ms when status execution is separate.

These regression blocks use an event wait, reproducing executor queue blockage;
they do not emulate a native call that holds the GIL for its entire duration.

Bounded full-stack observations all kept motors disabled and the gantry held:

| Retained stem | Observation | Publisher max gap | Max IMU publish-call duration | Outcome |
|---|---:|---:|---:|---|
| `priority-headless-1` | 32 s from launch | 141.9 ms | 0.18 ms | Safety clear; 2,789 WBC solves |
| `priority-gui-1` | 32 s from launch | 341.2 ms | 331.3 ms | Safety clear; 2,661 WBC solves |
| `priority-gui-2` | 32 s from launch; concurrent regression tests during startup | 243.7 ms | 233.8 ms | Safety IMU age 253 ms; no WBC solves |

`priority-headless-1` also overlapped the short diagnostic regression during its
initial startup. These records are not controlled idle-performance comparisons.
All raw CPU/IMU logs and failed results are retained. An external Linux readback
of live TIDs is saved as `priority-headless-1.readback.json`, independently
confirming the configured hierarchy and affinity.

In `priority-gui-1`, the 331 ms publish-call maximum first appears in diagnostics
at observer t=5.315 s, before the first observed Safety diagnostic at t=6.647 s.
Safety being clear later is not evidence it tolerates a 331 ms steady-state
sensor outage. Final reported physics real-time factors were 0.981, 0.975 and
0.883 respectively; no timing or walking improvement is established.

**The requested scheduling hierarchy is applied; intermittent IMU delays remain
unresolved.** Higher CPU priority cannot remove an internal blocking publish,
in-process GIL/lock dependency or an actual sensor outage. Safety and source-time
validation were not relaxed. No simulator or child process is left running;
intentional SIGINT teardown logs are retained and are not walking outcomes.
