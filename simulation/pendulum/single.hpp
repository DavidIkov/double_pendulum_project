
#pragma once

#include "mathcpp/rk4.hpp"
#include "mathcpp/vector.hpp"
#include "pendulum.hpp"
#include "simulation.hpp"

template <>
class Simulation<1> {
public:
    Simulation(Pendulum const& pendulum);

    mathcpp::Vector2F GetPendulumPosition() const;
    Pendulum const& GetPendulum() const;

    float GetMaxRadius() const;

    void Step(float dt);

private:
    Pendulum pendulum_;
    mathcpp::RK4<float, mathcpp::Vector2F, Pendulum const&> rk_;
};
