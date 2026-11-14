# Goal-shot / ML transition handoff — 2026-11-13

## Start here

Read [the dated test report](reports/2026-11-13-goal-shot.md), then [the ball-chase guide](../simulation/humanoid_ball_chase/README.md) and [the controller transition notes](model_based_walking_design.md#22-goal-shot-validation-and-ml-transition-2026-11-13). They contain implementation details, thresholds, results and artifact locations; this file only records continuation decisions.

The user intends to learn walking/turning/kicking with ML. The current model controller is a task-flow baseline. Do not spend the next session maximizing MPC shot precision unless requested. Preserve visual tracking, last reliable ball position, task supervision, independent safety and command expiry when replacing action execution. A trained policy and compatible adapter are still required.

## Checkout and publication boundary

Work was performed over SSH as `td@100.71.186.19` in `/home/darin/Documents/Github/embedded-vio`, branch `vio-test`, implementation baseline `9f32cc41ba93d9383f37e619ce460b2882260dda`. The latest publication request is for Markdown updates. These documents describe a tested, dirty working tree; the associated recent Python/C++/launch changes are not included in this documentation commit. Inspect `git status --short --branch` before continuing. A fresh checkout of this documentation commit alone does not reproduce the shot implementation.

`origin` in that checkout points to the local sibling `/home/darin/Documents/Github/embedded-sim-experimental`. The sibling's GitHub origin is `git@github.com:triton-droids/embedded.git`. Do not push to or reset the sibling's checked-out `sim-experimental-cyclonedds` branch by accident. Preserve `.deps`, `rosenv`, all existing dirty changes and simulation evidence. No entity should write commands alongside the active controller.

## Remaining work

- Timing: localize intermittent expired commands with correlated source, input, compute, publish, receive and deadline timestamps; independent maxima do not measure C++ relay residence. Do not increase the 80ms command lifetime or relax protection to obtain a success.
- Ball assumption: the last saved target assumes a stationary ball. One trial nudged it before kicking; distinguish that from detection confidence. Measurement-only MuJoCo truth must stay outside ball control.
- ML: first define observation/action units, axes, joint order, gains, update rate and acknowledgement/status semantics. Choose explicitly between replacing only MPC and replacing the whole low-level MPC/gait/WBC action controller. Keep one active command producer and fail closed on stale policy output.

For debugging, use the `diagnose` skill and the report's existing reproducer/measurements. For a documentation-only continuation no new motion test is required. No simulation or Isaac detector is left running; the pre-existing buildkit container is retained. No physical robot, CAN, firmware or flashing validation was performed.
