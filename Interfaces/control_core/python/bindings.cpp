#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "control_core/api.hpp"

namespace py = pybind11;
using namespace control_core;

PYBIND11_MODULE(control_core_py, m) {
  m.doc() = "control_core Python bindings (pybind11)";

  // Enum
  py::enum_<Mode>(m, "Mode")
      .value("Velocity", Mode::Velocity)
      .value("Position", Mode::Position)
      .value("Motion",   Mode::Motion)
      .value("Enable",   Mode::Enable)
      .value("Disable",  Mode::Disable);

  // JointData
  py::class_<JointData>(m, "JointData")
      .def(py::init<>())
      .def_readwrite("pos", &JointData::pos)
      .def_readwrite("vel", &JointData::vel)
      .def_readwrite("eff", &JointData::eff);

  // State (dict[str, JointData])
  py::class_<State>(m, "State")
      .def(py::init<>())
      .def_readwrite("joints", &State::joints);

  // Command
  py::class_<Command>(m, "Command")
      .def(py::init<>())
      .def_readwrite("mode", &Command::mode)
      .def_readwrite("position", &Command::position)
      .def_readwrite("velocity", &Command::velocity)
      .def_readwrite("acceleration", &Command::acceleration)
      .def_readwrite("torque", &Command::torque)
      .def_readwrite("kp", &Command::kp)
      .def_readwrite("kd", &Command::kd);

  // Output
  py::class_<Output>(m, "Output")
      .def(py::init<>())
      .def_readwrite("joint_name", &Output::joint_name)
      .def_readwrite("commands", &Output::commands);

  // Controller
  py::class_<Controller>(m, "Controller")
      .def(py::init<>())
      .def("configure", &Controller::configure)
      .def("reset_hold", &Controller::reset_hold, py::arg("value") = 0.0)
      .def("step", &Controller::step);
}
