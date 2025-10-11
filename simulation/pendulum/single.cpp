
#include "single.hpp"

static mathcpp::Vector2F CalculateRK4(float const& t,
                                      const mathcpp::Vector2F& state,
                                      Pendulum const& pendulum) {
    float l = pendulum.length_;
    float m = pendulum.mass_;
    float g = 9.80665f;
    float o = state[0], v = state[1];

    float md = -g / l * std::sin(o);

    // v1d -= pendulums[0].dumping_mult_ * v1;
    // v2d -= pendulums[1].dumping_mult_ * v2;

    return {v, md};
}

Simulation<1>::Simulation(Pendulum const& pendulum)
    : pendulum_(pendulum),
      rk_(0, mathcpp::Vector2F{pendulum_.angle_, 0}, pendulum_, &CalculateRK4) {
}

mathcpp::Vector2F Simulation<1>::GetPendulumPosition() const {
    return {std::sin(pendulum_.angle_) * pendulum_.length_,
            -std::cos(pendulum_.angle_) * pendulum_.length_};
}
Pendulum const& Simulation<1>::GetPendulum() const { return pendulum_; }

float Simulation<1>::GetMaxRadius() const { return pendulum_.length_; }

void Simulation<1>::Step(float dt) {
    mathcpp::Vector2F state = rk_.Step(dt);
    pendulum_.angle_ = state[0];
}
