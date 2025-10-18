#include "double.hpp"

#include <stdexcept>

mathcpp::Vector4F Simulation<2>::CalculateRK4(
    float const& t, const mathcpp::Vector4F& state,
    Simulation<2> const& simulation, mathcpp::Vector4F const& platform_state) {
    float l0 = simulation.pendulums_[0].length_,
          l1 = simulation.pendulums_[1].length_;
    float m0 = simulation.pendulums_[0].mass_,
          m1 = simulation.pendulums_[1].mass_;
    float g = simulation.gravity_;
    float o0 = state[0], o1 = state[1], v0 = state[2], v1 = state[3];
    float pvx = platform_state[0], pvy = platform_state[1],
          pax = platform_state[2], pay = platform_state[3];

    /*
     *a0*x+b0*y=c0
     *a1*x+b1*y=c1
     *
     * x=(c0-b0*y)/a0
     * a1*c0-b0*a1*y+b1*a0*y=c1*a0
     *
     * y=(a0*c1-a1*c0)/(b1*a0-b0*a1)
     * x=(c0-b0*y)/a0
     */

    float a0 = l0 * l0 * (m0 + m1), b0 = m1 * l0 * l1 * std::cos(o0 - o1),
          c0 = l0 * v0 * (m0 + m1) * (pvy * std::cos(o0) - pvx * std::sin(o0)) -
               m1 * l0 * l1 * v0 * v1 * std::sin(o0 - o1) -
               g * l0 * std::sin(o0) * (m0 + m1);
    c0 -= l0 * (m0 + m1) *
              (pax * std::cos(o0) - pvx * v0 * std::sin(o0) +
               pay * std::sin(o0) + pvy * v0 * std::cos(o0)) +
          m1 * l0 * l1 * v1 * std::sin(o0 - o1) * (v1 - v0);

    float a1 = m1 * l0 * l1 * std::cos(o0 - o1), b1 = m1 * l1 * l1,
          c1 = m1 * l1 * v1 * (pvy * std::cos(o1) - pvx * std::sin(o1)) +
               m1 * l0 * l1 * v0 * v1 * std::sin(o0 - o1) -
               m1 * g * l1 * std::sin(o1);

    c1 -= m1 * l1 *
              (pax * std::cos(o1) - pvx * v1 * std::sin(o1) +
               pay * std::sin(o1) + pvy * v1 * std::cos(o1)) +
          m1 * l0 * l1 * v0 * std::sin(o0 - o1) * (v1 - v0);

    float v1d = (a0 * c1 - a1 * c0) / (b1 * a0 - b0 * a1);
    float v0d = (c0 - b0 * v1d) / a0;

    v0d -= simulation.pendulums_[0].dumping_mult_ * v0;
    v1d -= simulation.pendulums_[1].dumping_mult_ * v1;

    return {v0, v1, v0d, v1d};
}

Simulation<2>::Simulation(std::initializer_list<Pendulum> pendulums,
                          float gravity)
    : pendulums_{pendulums.begin()[0], pendulums.begin()[1]},
      gravity_(gravity),
      rk_(0,
          mathcpp::Vector4F{pendulums_[0].angle_, pendulums_[1].angle_, 0, 0},
          *this, &CalculateRK4) {}

mathcpp::Vector2F Simulation<2>::GetPendulumPosition(size_t ind) const {
    if (ind == 0) {
        return {std::sin(pendulums_[0].angle_) * pendulums_[0].length_,
                -std::cos(pendulums_[0].angle_) * pendulums_[0].length_};
    } else if (ind == 1) {
        return GetPendulumPosition(0) +
               mathcpp::Vector2F{
                   std::sin(pendulums_[1].angle_) * pendulums_[1].length_,
                   -std::cos(pendulums_[1].angle_) * pendulums_[1].length_};
    } else
        throw std::runtime_error("unknown pendulum index");
}

std::array<Pendulum, 2> const& Simulation<2>::GetPendulums() const {
    return pendulums_;
}

Pendulum const& Simulation<2>::GetPendulum(size_t ind) const {
    return pendulums_[ind];
}

float Simulation<2>::GetMaxRadius() const {
    return pendulums_[0].length_ + pendulums_[1].length_;
}

void Simulation<2>::Step(float dt, mathcpp::Vector4F platform_state) {
    mathcpp::Vector4F state = rk_.Step(dt, platform_state);
    pendulums_[0].angle_ = state[0];
    pendulums_[1].angle_ = state[1];
}
