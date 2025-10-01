#include "double.hpp"

#include "mathcpp/matrix.hpp"

static mathcpp::Vector4F CalculateRK4(float const& t,
                                      const mathcpp::Vector4F& state,
                                      std::array<Pendulum, 2>& pendulums) {
    float l1 = pendulums[0].length_, l2 = pendulums[1].length_;
    float m1 = pendulums[0].mass_, m2 = pendulums[1].mass_;
    float g = 9.80665f;  // TODO
    float o1 = state[0], o2 = state[1], v1 = state[2], v2 = state[3];

    float a1 = l1 * l1 * (m1 + m2), b1 = m2 * l1 * l2 * std::cos(o1 - o2),
          c1 = m2 * v2 * v2 * l1 * l2 * std::sin(o1 - o2) +
               l1 * g * std::sin(o1) * (m1 + m2),
          a2 = m2 * l2 * l2, b2 = m2 * l1 * l2 * std::cos(o1 - o2),
          c2 = -m2 * v1 * v1 * l1 * l2 * std::sin(o1 - o2) +
               l2 * m2 * g * std::sin(o2);
    float v2d = (b2 * c1 - c2 * a1) / (a2 * a1 - b1 * b2),
          v1d = -(v2d * b1 + c1) / a1;

    v1d -= pendulums[0].dumping_mult_ * v1;
    v2d -= pendulums[1].dumping_mult_ * v2;

    return {v1, v2, v1d, v2d};
}

Simulation<2>::Simulation(std::initializer_list<Pendulum> pendulums)
    : pendulums_{pendulums.begin()[0], pendulums.begin()[1]},
      rk_(0,
          mathcpp::Vector4F{pendulums_[0].angle_, pendulums_[1].angle_, 0, 0},
          pendulums_, &CalculateRK4) {}

std::array<mathcpp::Vector2F, 2> Simulation<2>::GetPendulumsPositions() const {
    mathcpp::Vector2F p0 = mathcpp::Mat2D_RotBaseF.RotateVectorC<0, 1>(
                               {0.f, -1.f}, pendulums_[0].angle_) *
                           pendulums_[0].length_;
    mathcpp::Vector2F p1 = p0 + mathcpp::Mat2D_RotBaseF.RotateVectorC<0, 1>(
                                    {0.f, -1.f}, pendulums_[1].angle_) *
                                    pendulums_[1].length_;
    return {p0, p1};
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

void Simulation<2>::Step(float dt) {
    mathcpp::Vector4F state = rk_.Step(dt);
    pendulums_[0].angle_ = state[0];
    pendulums_[1].angle_ = state[1];
}
