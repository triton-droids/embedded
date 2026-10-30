# control_core

Active ROS-independent C++ library used by `cpp_control_node`.

- `include/control_core/api.hpp`, `src/api.cpp`: per-joint command selection,
  hold-last storage and optional advisory safety features.
- `include/control_core/operation.hpp`, `src/operation.cpp`: guarded operation
  state machines.
- `python/bindings.cpp`: standalone pybind11 interface to the command core.

The ROS adapter builds these sources directly through
[`motor_control_hybrid/CMakeLists.txt`](../../humanoid_control/motor_control_hybrid/CMakeLists.txt)
and installs these public headers. This library does not implement MPC or WBC.

For a standalone C++ build without ROS or Python bindings, from the repo root:

```bash
cmake -S libraries/control_core -B /tmp/control-core-build \
  -DCONTROL_CORE_BUILD_PYTHON=OFF
cmake --build /tmp/control-core-build
```

Python bindings are opt-in (`-DCONTROL_CORE_BUILD_PYTHON=ON`) and require
pybind11 and Python development headers. The former HTTP client examples are
retained in [legacy/http_gateway_examples](../../legacy/http_gateway_examples/README.md).
