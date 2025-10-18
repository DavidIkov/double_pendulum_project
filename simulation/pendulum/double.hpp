
#pragma once
#include <array>

#include "mathcpp/rk4.hpp"
#include "mathcpp/vector.hpp"
#include "pendulum.hpp"
#include "simulation.hpp"

template <>
class Simulation<2> {
public:
    Simulation(std::initializer_list<Pendulum> pendulums,
               float gravity = 9.80665f);

    mathcpp::Vector2F GetPendulumPosition(size_t ind) const;
    std::array<Pendulum, 2> const& GetPendulums() const;
    Pendulum const& GetPendulum(size_t ind) const;

    float GetMaxRadius() const;

    void Step(float dt, mathcpp::Vector4F platform_acceleration = 0.f);

private:
    static mathcpp::Vector4F CalculateRK4(
        float const& t, const mathcpp::Vector4F& state,
        Simulation<2> const& simulation,
        mathcpp::Vector4F const& platform_state);

    std::array<Pendulum, 2> pendulums_;
    float gravity_;
    mathcpp::RK4<float, mathcpp::Vector4F, Simulation<2> const&,
                 mathcpp::Vector4F const&>
        rk_;
};
