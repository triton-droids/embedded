# ROS 2 tracking policy timing results

Measured on 2026-10-07 on an aarch64 Jetson Orin Nano, Ubuntu 22.04,
ROS 2 Humble / rmw_fastrtps_cpp. Model SHA256:
`90fc691abbf43b89a55956493da6ab19c59105f9167f29f8f93fc317b68c9309`.
The ESP32-S3 published physical IMU data; joint feedback was a placeholder.
There were no connected motors, SDK or CAN node. These are bench measurements,
not physical robot stability or actuator-latency measurements.

The custom observers were omitted. Standard `ros2 topic hz --window 5000`
captured each scenario for 60 seconds; one-shot `ros2 topic echo` retrieved the
node's existing status. Native CLI min/max intervals are rounded to milliseconds.

| Scenario | Topic | Average Hz | Max receiver interval ms | Std dev ms | Final window samples |
| --- | --- | ---: | ---: | ---: | ---: |
| Physical IMU, zero feedback | /policy/target_angles | 50.000 | 24 | 0.49 | 2985 |
| Physical IMU, C++/fake retry | /policy/target_angles | 50.000 | 28 | 1.10 | 3038 |
| Physical IMU, C++/fake retry | /policy/motor_commands | 50.001 | 25 | 0.53 | 2987 |
| Physical IMU, C++/fake retry | /imu/data_raw | 200.024 | 16 | 0.72 | 5000 |

The final 200 Hz IMU window covers approximately 25 seconds. Policy/C++ samples
cover approximately one minute. Both successful runs ended in running state
with no fault and no callback work duration above 20 ms.

| Scenario | Inference p99/max ms | Callback work p99/max ms | Callback entry interval p99/max ms |
| --- | --- | --- | --- |
| Zero feedback | 0.860 / 2.370 | 2.701 / 4.354 | 20.503 / 21.930 |
| C++/fake retry | 1.378 / 4.010 | 4.481 / 9.045 | 21.908 / 24.786 |

**Failed first C++ capture:** IMU timeout latched after 209 successful policy
iterations. The native target-angle tool received only 101 messages before
silence. C++ continued at about 49.918 Hz, including fallback/disable commands;
this is not a successful policy test. No simultaneous IMU hz capture was present
to distinguish a publishing interruption from delivery/execution delay. The
retry added an IMU hz measurement and finished normally. No production code or
watchdog threshold changed. The transient timeout's cause remains unresolved.

Earlier instrumented runs demonstrated observer-induced long intervals: an
85.163 ms generation-2 GC pause in the custom observer overlapped a 103.725 ms
receiver gap, while publisher headers remained about 21 ms apart and RMW transit
was 0.213 ms for that sample. A controlled run with observer GC disabled had a
25.882 ms maximum receiver interval. This explains that measured long pause;
it does not prove all delays came from the observer. Other traces showed actual
publisher jitter and involuntary context switches during inference.

Average 50 Hz therefore does not establish a strict 20 ms deadline. Native hz
measures receiving rate, and internal callback timing excludes complete
sensor-to-actuator latency. The last reference frame is held after 5.96 seconds;
long runs assess execution timing rather than continued walking tracking.

See [startup and test procedure](tracking_policy_ros2.md#native-ros-2-frequency-test-and-logs)
for commands and the status checks required to interpret the frequency results.
