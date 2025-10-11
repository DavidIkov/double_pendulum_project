
#include "pendulum/single.hpp"

#include <stdio.h>

#include <chrono>
#include <cmath>
#include <random>
#include <thread>

#include "renderer/renderer.hpp"

int main(int, char**) {
    std::mt19937 rand_gen((std::random_device())());
    std::uniform_real_distribution<float> angle_distr(M_PI / 2, -M_PI / 2),
        length_distr(20, 80), mass_distr(2, 20), damping_distr(0, 0.3f),
        noise_distr(0, 0.1f);

    Simulation<1> simulation({angle_distr(rand_gen), length_distr(rand_gen),
                              mass_distr(rand_gen), damping_distr(rand_gen)});
    float radius = simulation.GetMaxRadius();

    {
        Pendulum const& pend = simulation.GetPendulum();
        printf(
            "Starting conditions:\nPendulum0: angle = %f, length = %f, "
            "mass = "
            "%f, damping = %f\n",
            pend.angle_, pend.length_, pend.mass_, pend.dumping_mult_);
    }

    ConsoleRenderer renderer({80 + 1, 40 + 1});

    while (1) {
        mathcpp::Vector2F p0 = simulation.GetPendulumPosition();

        renderer.Draw(renderer.GetSize() / 2);
        renderer.Draw((p0 / radius).Clamp({-1.f, -1.f}, {1.f, 1.f}));

        renderer.SwapBuffers();
        renderer.ClearBackBuffer();

        for (size_t step = 0; step < 10; ++step) simulation.Step(0.03f);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
