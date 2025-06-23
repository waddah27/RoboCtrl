#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "trajectory.hh"

namespace py = pybind11;

PYBIND11_MODULE(trajectory, m) {
    py::class_<TrajectoryState>(m, "TrajectoryState")
        .def(py::init<double, double, double, double>(),
             py::arg("xt") = 0.0, py::arg("xd") = 0.0, py::arg("vmax") = 0.0, py::arg("step") = 0.0)
        .def_readwrite("x_t", &TrajectoryState::x_t)
        .def_readwrite("x_d", &TrajectoryState::x_d)
        .def_readwrite("v_max", &TrajectoryState::v_max)
        .def_readwrite("step", &TrajectoryState::step);

    py::class_<LinearStepPlanner>(m, "LinearStepPlanner")
        .def(py::init<>())
        .def(py::init<TrajectoryState>())
        .def("plan_trajectory", &LinearStepPlanner::plan_trajectory)
        .def("get_next_setpoint", &LinearStepPlanner::get_next_setpoint)
        .def("print_summary", &LinearStepPlanner::print_summary);

    py::class_<TrapezoidalPlanner>(m, "TrapezoidalPlanner")
        .def(py::init<>())
        .def(py::init<double, double, double, double, double, bool>())
        .def("update", &TrapezoidalPlanner::update)
        .def("is_done", &TrapezoidalPlanner::is_done);

    py::class_<TrapezoidalPlannerND>(m, "TrapezoidalPlannerND")
        .def(py::init<>())
        .def(py::init<const std::vector<double>&, const std::vector<double>&, double, double, double, bool>())
        .def("update", &TrapezoidalPlannerND::update)
        .def("is_done", &TrapezoidalPlannerND::is_done);
}