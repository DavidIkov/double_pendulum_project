
#include "single.hpp"

mathcpp::Vector2F Simulation<1>::CalculateRK4(
    float const& t, const mathcpp::Vector2F& state,
    Simulation<1> const& simulation,
    const mathcpp::Vector2F& platform_acceleration) {
    float l = simulation.pendulum_.length_;
    float m = simulation.pendulum_.mass_;
    float g = simulation.gravity_;
    float o = state[0], v = state[1];

    float pax = platform_acceleration[0], pay = platform_acceleration[1];

    float vd = -(pax * std::cos(o) + pay * std::sin(o) + g * std::sin(o)) / l;

    vd -= simulation.pendulum_.dumping_mult_ * v;

    return {v, vd};
}

Simulation<1>::Simulation(Pendulum const& pendulum, float gravity)
    : pendulum_(pendulum),
      gravity_(gravity),
      rk_(0, mathcpp::Vector2F{pendulum_.angle_, 0}, *this, &CalculateRK4) {}

mathcpp::Vector2F Simulation<1>::GetPendulumPosition() const {
    return {std::sin(pendulum_.angle_) * pendulum_.length_,
            -std::cos(pendulum_.angle_) * pendulum_.length_};
}
Pendulum const& Simulation<1>::GetPendulum() const { return pendulum_; }

float Simulation<1>::GetMaxRadius() const { return pendulum_.length_; }

void Simulation<1>::Step(float dt, mathcpp::Vector2F platform_acceleration) {
    mathcpp::Vector2F state = rk_.Step(dt, platform_acceleration);
    pendulum_.angle_ = state[0];
}
