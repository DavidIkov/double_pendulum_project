#include "double.hpp"

#include "mathcpp/matrix.hpp"

DoublePendulum::DoublePendulum(Pendulum pendulum0, Pendulum pendulum1)
    : pendulum0_(pendulum0),
      pendulum1_(pendulum1),
      rk_(0, mathcpp::Vector4F{pendulum0_.angle_, pendulum1_.angle_, 0, 0},
          &pendulum0_,
          [](const float& t, const mathcpp::Vector4F& state,
             Pendulum* const& pends) -> mathcpp::Vector4F {
              float l1 = pends[0].length_, l2 = pends[1].length_;
              float m1 = pends[0].mass_, m2 = pends[1].mass_;
              float g = 9.80665f;  // TODO
              float o1 = state[0], o2 = state[1], v1 = state[2], v2 = state[3];
              float a1 = l1 * l1 * (m1 + m2),
                    b1 = m2 * l1 * l2 * std::cos(o1 - o2),
                    c1 = m2 * v2 * v2 * l1 * l2 * std::sin(o1 - o2) +
                         l1 * g * std::sin(o1) * (m1 + m2),
                    a2 = m2 * l2 * l2, b2 = m2 * l1 * l2 * std::cos(o1 - o2),
                    c2 = -m2 * v1 * v1 * l1 * l2 * std::sin(o1 - o2) +
                         l2 * m2 * g * std::sin(o2);
              float v2d = (b2 * c1 - c2 * a1) / (a2 * a1 - b1 * b2),
                    v1d = -(v2d * b1 + c1) / a1;

              return {v1, v2, v1d, v2d};
          }) {}

mathcpp::Vector2F DoublePendulum::GetPendulum0Pos() const {
    return mathcpp::Mat2D_RotBaseF.RotateVectorC<0, 1>({0.f, -1.f},
                                                       pendulum0_.angle_) *
           pendulum0_.length_;
}
mathcpp::Vector2F DoublePendulum::GetPendulum1Pos() const {
    return GetPendulum0Pos() + mathcpp::Mat2D_RotBaseF.RotateVectorC<0, 1>(
                                   {0.f, -1.f}, pendulum1_.angle_) *
                                   pendulum1_.length_;
}

float DoublePendulum::GetMaxRadius() const {
    return pendulum0_.length_ + pendulum1_.length_;
}

void DoublePendulum::Step(float dt) {
    mathcpp::Vector4F state = rk_.Step(dt);
    pendulum0_.angle_ = state[0];
    pendulum1_.angle_ = state[1];
}
