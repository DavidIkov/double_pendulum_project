
#pragma once

#include "mathcpp/rk4.hpp"
#include "mathcpp/vector.hpp"
#include "pendulum.hpp"
#include "simulation.hpp"

template <>
class Simulation<1> {
public:
    Simulation(Pendulum const& pendulum, float gravity = 9.80665f);

    mathcpp::Vector2F GetPendulumPosition() const;
    Pendulum const& GetPendulum() const;

    float GetMaxRadius() const;

    void Step(float dt, mathcpp::Vector2F platform_acceleration = 0.f);

private:
    static mathcpp::Vector2F CalculateRK4(
        float const& t, const mathcpp::Vector2F& state,
        Simulation<1> const& simulation,
        const mathcpp::Vector2F& platform_acceleration);

    Pendulum pendulum_;
    float gravity_;
    mathcpp::RK4<float, mathcpp::Vector2F, Simulation<1> const&,
                 mathcpp::Vector2F>
        rk_;
};
