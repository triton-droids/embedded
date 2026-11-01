# Dedicated IMU thread, KeepLast 2, and zero-copy capability

## Implemented

Body IMU publication defaults to a dedicated `SingleThreadedExecutor` and a
named `sim_body_imu_executor` thread. Diagnostics run on the other executor in
the normal simulator composition. Both the node parameter and simulation launch
default to `dedicated_imu_executor=true`; the chase composition already selected
this mode. Physics sampling still runs on `mujoco_physics` at the IMU scheduling
role. The priority order remains IMU/Safety > WBC > other.

`/imu/status` now exposes `publish_thread`, `publish_native_tid`, and `qos_depth`.
This is callback isolation, not Python GIL or process isolation: native DDS code
that holds the GIL can still delay Python threads. Do not treat a dedicated
executor as a cure for the previously measured Fast DDS lock waits.

Application topic writers/readers in the running simulation, camera/vision,
ball controls/planner, policy, WBC, C++ control, Safety and Web chain now use
KeepLast=2, including endpoints formerly at depth 1, 3, 5, 10 and 20. History
depth is not the message-filter synchronizer queue size. ROS-internal discovery,
rosout/parameter-event endpoints, and service histories are outside this topic
tuning. Offline historical DDS benchmark fixtures retain their explicit test
settings. Monitoring scripts now request depth 2 too.

Reliability and durability choices are retained. Estop remains reliable and
transient-local, with history depth now 2. Source timestamps, angular/joint
velocities, 250 ms IMU expiry, command expiry and controller gains are retained.
Kd still consumes measured joint velocity; it does not differentiate queued
samples. History depth 2 does not provide a scheduling deadline guarantee.

## What is not zero-copy

The existing Python application chain has **not** been changed to end-to-end
zero-copy. A C++ capability probe actually creates each message type at depth 2
and asks `Publisher::can_loan_messages()`. It borrows/publishes a loan only when
the middleware advertises native support; it does not count rclcpp's allocating
fallback as a native loan. Research messages use `/research/zero_copy/*`, never
control topics. A successful publisher loan alone is not proof of zero-copy
delivery to a subscriber.

| Message | Fast DDS 2.14.5 with Data Sharing AUTO | Private Cyclone DDS build |
| --- | --- | --- |
| `sensor_msgs/Imu` | cannot loan | cannot loan |
| `sensor_msgs/Image` | cannot loan | cannot loan |
| `sensor_msgs/JointState` | cannot loan | cannot loan |
| `motor_control_interfaces/MotorCommand` | cannot loan | cannot loan |
| `std_msgs/Float32` | native loan valid | cannot loan |

The Fast DDS run enabled `ROS_DISABLE_LOANED_MESSAGES=0` and
`RMW_FASTRTPS_USE_QOS_FROM_XML=1` with the explicit
`publish_probe/data_sharing_auto.xml` profile. Therefore this is not merely a
negative result from leaving data sharing disabled. The Cyclone run used domain
209; an initial domain 233 attempt was invalid because calculated UDP ports
exceeded 65535. It was corrected before collecting the valid capability result.
See `results/zero-copy-{fastdds,cyclone}-capability.txt`.

`Imu` includes the dynamically sized `Header.frame_id`; Image, JointState and
MotorCommand also contain unbounded sequences or strings. In addition, the
current rclpy publish/subscribe APIs convert application objects rather than
exposing the native borrowed-buffer interface used by the C++ probe. Merely
selecting shared-memory transport or setting `use_intra_process_comms` on these
separate Python processes cannot establish end-to-end zero-copy.

Fast DDS documents that its zero-copy path combines data sharing with writer
and reader loans and requires plain/bounded types:
[official Fast DDS 2.14 zero-copy requirements](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/use_cases/zero_copy/zero_copy.html).

A true zero-copy control path would require fixed-layout messages with explicit
frame/joint registry identities and native C++ publishing/consuming code, or a
separate shared-memory protocol with buffer-lifetime ownership. A conversion
bridge back to standard Python ROS messages introduces copies at that boundary.
That architectural rewrite is not implemented or claimed by this change.
The AUTO XML is a capability experiment only and is not loaded by `run_chase.sh`.

## Validation

`motor_control_hybrid` and `humanoid_vision` rebuilt sequentially. The selected
Python regression batch passed 51 tests (one gated ROS integration test skipped);
6 transport/IMU/QoS tests passed after adding thread-ID assertions.
Both `cpp_state_watchdog_test` and `core_operation_test` passed. The IMU executor
regression confirms that its timer continues publishing while another executor
callback is blocked, using the new default separation.
