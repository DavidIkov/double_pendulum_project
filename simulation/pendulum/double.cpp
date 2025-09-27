#include "double.hpp"

#include "mathcpp/matrix.hpp"

DoublePendulum::DoublePendulum(Pendulum pendulum0, Pendulum pendulum1)
    : pendulum0_(pendulum0), pendulum1_(pendulum1) {}

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
