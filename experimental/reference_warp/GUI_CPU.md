# GUI, CPU and MuJoCo scheduling investigation

Measured on 2026-10-31 on the current Ubuntu workstation: i7-1165G7,
4 physical cores / 8 logical CPUs, Intel Iris Xe, Mesa 25.2.8 accelerated
OpenGL, MuJoCo 3.3.7. CPU governor reported `powersave`; no governor, affinity,
priority, safety deadline, physics timestep or production scheduler was changed.

CPU percentages below use **100% = one logical CPU**, not the whole machine.
Eight logical CPUs do not provide eight independent physical cores. These are
short desktop observations, not an isolated real-time or thermal benchmark.

## Component isolation

`gui_cpu_benchmark.py` uses the actual full D435/mount/football model, real
`SimWorld`, actual `ViewerProcess`, and actual `SimCameraNode` at 424x240 RGB-D,
6 Hz. The robot stays held with disabled leg motors. Policy, WBC, YOLO and other
ROS nodes are absent; there are no image subscribers. CPU totals include the
benchmark's descendants, but exclude compositor/Xorg CPU and GPU utilization.
After a two-second warmup each observation lasts eight seconds. Four normal
cases have two observations in reversed order; the two artificial scheduler
controls have one each. All records are retained in `results/cpu-*.json` and
corresponding logs. Component GUI repetitions recorded real MuJoCo X11 windows.

| Configuration | Process-tree CPU | Physical steps/s | Real-time factor |
|---|---:|---:|---:|
| Physics only, current pacing | 8.1–8.7% | 200.0–200.1 | 1.000 |
| Physics + GUI | 17.6–20.3% | 200.0–200.1 | 1.000 |
| Physics + RGB-D | 11.4–11.5% | 200.0 | 1.000 |
| Physics + GUI + RGB-D | 21.8–22.7% | 200.0 | 1.000 |
| Artificial: remove pacing | 100.1% | 18,828 | 94.14 |
| Artificial: add 6 ms CPU work/step | 100.0% | 163.2 | 0.816 |

The GUI child itself used 8.9–10.2% of one logical CPU. Normal component
observations had a maximum step-start gap of 5.2–8.2 ms and step wall-time p99
of 0.59–0.72 ms. These held-state numbers do not establish walking/contact-load
performance. The fast unpaced case reaches a different physical trajectory and
CPU power/cache state; its step rate is not a transferable speedup estimate.

The overloaded control uses `time.thread_time()` and actual Python busy work,
not a sleep. It intentionally holds the Python GIL during the injected work;
it demonstrates saturation/slowdown, not a reproduction of the IMU fault.

## What the current scheduler does

`world.py:SimWorld._run` budgets 5 ms per physical step (200 Hz). It uses
`time.monotonic()` to pace steps, sleeps until the next deadline when early,
and discards overdue deadlines rather than replaying historical steps. On
overrun it sets the next deadline to `max(step_start + dt, finish)` and calls
`sleep(0)` when already late. This yields execution but reserves no CPU budget
for other threads. A sustained expensive step can therefore occupy one core
while simulated time falls behind wall time. IMU/command age is still measured
against host time; slowing physics does not extend those deadlines.

The actual physics path is standard CPU `mujoco.mj_step`; this repository does
not configure an internal MuJoCo worker pool or use MJX/GPU physics. More cores
therefore do not automatically divide this single-world step among all CPUs.
MuJoCo's C calls release the GIL for the C call itself; surrounding Python code
still uses the interpreter lock. See the [official Python documentation](https://mujoco.readthedocs.io/en/3.3.7/python.html)
and [simulation documentation](https://mujoco.readthedocs.io/en/3.3.7/programming/simulation.html).

## GUI and camera are separate costs

`viewer.py` starts a spawned child with separate `MjData`. The parent copies
integration state under `world.lock` and passes only the latest snapshot via a
bounded queue. The child performs `mj_forward` and passive-viewer `sync` outside
the live physics lock. Parent updates and child synchronization are nominally
5 Hz. **This is not a measured or enforced native window redraw frame cap.**
The passive viewer has its own render thread; OpenGL acceleration still needs
CPU scene work, driver work and window composition.

`camera_node.py` uses its own thread in the simulator process. It builds both
RGB and depth scenes while holding `world.lock`, then renders and publishes
outside the lock. It still shares the simulator's process, GIL, CPU and memory
resources. `use_viewer:=false` removes only the interactive viewer; RGB-D
rendering and vision inference remain active. The measured backend is hardware
accelerated, not `llvmpipe` software rendering. GPU load/bandwidth/power were not
measured, so CPU totals do not quantify all rendering costs.

## Full-stack comparison

`stack_cpu_probe.py` serially launches the actual experimental chase launch with
RGB-D, YOLO, policy, WBC, C++, Safety and Web retained. `start_chase:=false` keeps
the gantry held and never requests ENABLE/release. Order is headless, GUI, GUI,
headless on isolated domains 211–214, network DDS enabled. Each launch is
observed for 32 seconds including startup; CPU averages use samples at 12–32 s.
The IMU observer is a separate process excluded from the launch-tree CPU total.
Safety and publisher diagnostics are recorded separately from observer latency.

| Record | Tree CPU, 12–32 s | Publisher max gap, whole observation | Last reported RTF | Outcome |
|---|---:|---:|---:|---|
| `cpu-stack-headless-1` | 249.9% | 106.1 ms | 1.000 | Safety clear; policy active; 2,690 WBC solves |
| `cpu-stack-gui-1` | 299.4% | 28.8 ms | 0.990 | Safety clear; policy active; 2,686 WBC solves |
| `cpu-stack-gui-2` | 300.4% | 25.4 ms | 0.990 | Safety IMU age 340 ms; chase stale-command fault; only 9 WBC solves |
| `cpu-stack-headless-2` | 273.5% | 145.9 ms | 1.001 | Safety clear; policy active; 2,644 WBC solves |

These four records are held timing observations, with **one faulted run kept**.
The faulted run's post-startup workload differs from the healthy runs; it cannot
serve as an equivalent active-controller CPU comparison. There is also desktop
and frequency variation between healthy observations. The observed stack CPU
difference is not an estimate of pure viewer cost; component isolation above
is the cleaner marginal-cost measurement.

On the healthy headless observations, approximate per-process CPU ranges were:
simulator 92.5–100.0%, YOLO/ball vision 57.3–63.7%, WBC 38.8–43.8%, policy
32.3–34.8%, Safety 12.4–13.5%, C++ 9.3–10.1%. The GUI child's full-stack CPU was
5.7% in both GUI observations. The difference from its component CPU emphasizes
that scene activity and runtime scheduling affect the cost. All sampled stack
processes used scheduler policy **0 (`SCHED_OTHER`)**, with affinity to all eight
logical CPUs. A ROS timer frequency or dedicated executor is not an OS deadline
guarantee. No real-time priority or CPU partitioning is currently configured.

The simulator process includes physics, sensor production, motor feedback,
odometry, DDS activity and Python executors. Its near-one-core CPU use must not
be attributed entirely to `mj_step`: the component physics-only process used
8–9%, and camera-only added about 3 percentage points. This comparison identifies
a substantial additional full-stack workload, not a function-level CPU profile.

The second GUI observation is particularly informative about IMU failure:

- First observed Safety status at observer t=6.993 s was already latched for
  IMU age 0.340 s. Publisher maximum gap remained 25.4 ms; the independent
  observer's whole-run maximum receipt gap was 24.0 ms and maximum source age
  26.3 ms. Physics stayed held and unpaused with no `control_fault`.
- The chase adapter also latched `stale_chase_command`. Its low iteration/solve
  counts are preserved, not relabeled as a healthy control run. There was no
  gantry release or walking exposure.
- These data narrow the discrepancy to consumer receipt/dispatch, middleware,
  or consumer-side scheduling/processing. They do **not** identify which native
  call, lock, GIL interval or kernel scheduling event caused it.
- Conversely, the first GUI observation's independent observer had a 339 ms
  receipt gap while Safety stayed clear and publisher gap was only 28.8 ms.
  Different subscribers can have different pauses. An observer's worst gap
  cannot be substituted for Safety's own receipt history.

Full-stack physics overrun counts were 24, 77, 88 and 29 in launch order. This
small cohort is consistent with more missed 5 ms budgets under GUI workload,
but startup and varying control state prevent a causal or reliability estimate.
The three clear observations do not cancel the faulted one or the earlier
headless failures.

## Interpretation

Closing the GUI reduces rendering overhead but does not remove the camera or
turn the application into a real-time process. Average machine CPU, per-process
CPU and worst-case callback/source gaps answer different questions. A process
can pause for 250 ms despite moderate average machine utilization. Conversely,
one occupied logical CPU is only 12.5% of this machine's aggregate logical-CPU
capacity. Affinity alone cannot remove in-process GIL/lock contention.

Previous held headless records in [IMU_TIMING.md](IMU_TIMING.md) already include
~294 ms publisher gaps and an IMU freshness trip with the GUI absent; disabling
camera also left a ~288 ms gap. Thus neither GUI nor RGB-D has been established
as the sole cause. The remaining intermittent stall is unresolved. These new
CPU measurements must not be called a safety fix or a walking-stability test.

All launched experiments are stopped after collection. SIGINT teardown logs
are retained separately from the measured runtime; abnormal shutdown messages
must not be counted as in-window scheduling failures without their timestamps.
Some processes reported exit -2 and vision reported exit -6 during deliberate
SIGINT teardown, after recording; clean shutdown is not claimed. No launched
simulator/viewer/policy/WBC process was left running. Both added probes passed
Python syntax checking and completed their actual bounded runs; no production
control changes were made in this investigation.

For a reproduction (from the repository root after sourcing
`scripts/activate_rosenv.sh`):

```bash
export PYTHONPATH="$PWD/experimental/reference_warp:$PYTHONPATH"
ROS_DOMAIN_ID=210 ROS_LOCALHOST_ONLY=0 MUJOCO_GL=egl \
  python experimental/reference_warp/gui_cpu_benchmark.py --case both \
  --output experimental/reference_warp/results/my-cpu-both.json
python experimental/reference_warp/stack_cpu_probe.py --gui false --domain 211 \
  --name my-cpu-stack-headless
```

Use a new result name and an unused ROS domain/Web port for subsequent runs.
