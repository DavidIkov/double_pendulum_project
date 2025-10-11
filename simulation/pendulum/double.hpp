
#pragma once
#include <array>

#include "mathcpp/rk4.hpp"
#include "mathcpp/vector.hpp"
#include "pendulum.hpp"
#include "simulation.hpp"

template <>
class Simulation<2> {
public:
    Simulation(std::initializer_list<Pendulum> pendulums);

    std::array<mathcpp::Vector2F, 2> GetPendulumsPositions() const;
    std::array<Pendulum, 2> const& GetPendulums() const;
    Pendulum const& GetPendulum(size_t ind) const;

    float GetMaxRadius() const;

    void Step(float dt);

private:
    std::array<Pendulum, 2> pendulums_;
    mathcpp::RK4<float, mathcpp::Vector4F, std::array<Pendulum, 2> const&> rk_;
};
