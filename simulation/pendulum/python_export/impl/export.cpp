#include "pybind11/pybind11.h"
#include "single.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pendulum_simulation, m) {
    py::class_<Pendulum>(m, "Pendulum")
        .def(py::init(
                 [](float angle, float length, float mass, float dumping_mult) {
                     return Pendulum{angle, length, mass, dumping_mult};
                 }),
             py::arg("angle") = 0.f, py::arg("length") = 1.f,
             py::arg("mass") = 1.f, py::arg("dumping_mult") = 0.f)
        .def_readwrite("angle_", &Pendulum::angle_)
        .def_readwrite("length_", &Pendulum::length_)
        .def_readwrite("mass_", &Pendulum::mass_)
        .def_readwrite("dumping_mult_", &Pendulum::dumping_mult_);
    py::class_<Simulation<1>>(m, "SinglePendulumSimulation")
        .def(py::init([](Pendulum const& pendulum, float gravity) {
                 return Simulation<1>(pendulum, gravity);
             }),
             py::arg("pendulum"), py::arg("gravity") = 9.80665f)
        .def("GetPendulumPosition",
             [](Simulation<1>& self) {
                 mathcpp::Vector2F pos = self.GetPendulumPosition();
                 return py::make_tuple(pos[0], pos[1]);
             })
        .def("GetPendulum", &Simulation<1>::GetPendulum)
        .def("GetMaxRadius", &Simulation<1>::GetMaxRadius)
        .def(
            "Step",
            [](Simulation<1>& self, float dt, py::tuple platform_acceleration) {
                self.Step(dt, {platform_acceleration[0].cast<float>(),
                               platform_acceleration[1].cast<float>()});
            },
            py::arg("dt"),
            py::arg("platform_acceleration") = py::make_tuple(0.f, 0.f));
}
