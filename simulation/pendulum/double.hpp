
#pragma once
#include "mathcpp/rk4.hpp"
#include "mathcpp/vector.hpp"

struct Pendulum {
    float angle_ = 0.f, length_ = 1.f, mass_ = 1.f;
};

class DoublePendulum {
public:
    DoublePendulum(Pendulum pendulum0, Pendulum pendulum1);

    mathcpp::Vector2F GetPendulum0Pos() const;
    mathcpp::Vector2F GetPendulum1Pos() const;

    float GetMaxRadius() const;

    void Step(float dt);

private:
    Pendulum pendulum0_, pendulum1_;
    mathcpp::RK4<float, mathcpp::Vector4F, Pendulum*> rk_;
};
