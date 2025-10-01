
#include <stdio.h>

#include <chrono>
#include <cmath>
#include <random>
#include <thread>
#include <vector>

#include "pendulum/double.hpp"

int main(int, char**) {
    std::mt19937 rand_gen((std::random_device())());
    std::uniform_real_distribution<float> angle_distr(M_PI / 2, M_PI * 1.5f),
        length_distr(20, 80), mass_distr(2, 20), damping_distr(0, 0.3f),
        noise_distr(0, 0.1f);

    Simulation<2> simulation{{angle_distr(rand_gen), length_distr(rand_gen),
                              mass_distr(rand_gen), damping_distr(rand_gen)},
                             {angle_distr(rand_gen), length_distr(rand_gen),
                              mass_distr(rand_gen), damping_distr(rand_gen)}};
    float radius = simulation.GetMaxRadius();

    {
        auto& pends = simulation.GetPendulums();
        printf(
            "Starting conditions:\nPendulum0: angle = %f, length = %f, mass = "
            "%f, damping = %f\nPendulum1: angle = %f, length = %f, mass = %f, "
            "damping = %f\n",
            pends[0].angle_, pends[0].length_, pends[0].mass_,
            pends[0].dumping_mult_, pends[1].angle_, pends[1].length_,
            pends[1].mass_, pends[1].dumping_mult_);
    }

    mathcpp::Vector2U boxes_amount{80, 40};

    mathcpp::Vector2F box_size = mathcpp::Vector2F(radius * 2) / boxes_amount;

    std::vector<bool> boxes_front(boxes_amount[0] * boxes_amount[1]);
    std::vector<bool> boxes_back = boxes_front;

    {  // PREPARE CONSOLE
        printf("TOP\n");
        for (size_t i = 0; i < boxes_amount[1]; ++i) printf("\n");
        printf("BOTTOM");
        printf("\r\e[%uA", (unsigned)boxes_amount[1]);
        fflush(stdout);
    }

    while (1) {
        std::fill(boxes_back.begin(), boxes_back.end(), 0);

        auto pendulums_poses = simulation.GetPendulumsPositions();
        mathcpp::Vector2F p0 = pendulums_poses[0];
        mathcpp::Vector2F p1 = pendulums_poses[1];

        p0 = {p0[0] + radius, radius - p0[1]},
        p1 = {p1[0] + radius, radius - p1[1]};

        mathcpp::Vector2U p0_box(p0 / box_size);
        mathcpp::Vector2U p1_box(p1 / box_size);

        for (size_t step = 0; step < 10; ++step) simulation.Step(0.03f);

        boxes_back[boxes_amount[0] / 2 +
                   boxes_amount[0] * boxes_amount[1] / 2] = 1;
        boxes_back[p0_box[0] + boxes_amount[0] * p0_box[1]] = 1;
        boxes_back[p1_box[0] + boxes_amount[0] * p1_box[1]] = 1;

        {  // CONSOLE RENDERING!!!!
            mathcpp::Vector2U cursor{0, 0};
            for (size_t ind = 0; ind < boxes_front.size(); ++ind)
                if (boxes_back[ind] != boxes_front[ind]) {
                    unsigned x = ind % boxes_amount[0],
                             y = ind / boxes_amount[0];
                    if (unsigned off = y - cursor[1]) printf("\e[%uB", off);
                    if (x) printf("\e[%uC", x);
                    if (boxes_back[ind])
                        printf("+");
                    else
                        printf(" ");
                    printf("\r");
                    boxes_front[ind] = boxes_back[ind];
                    cursor = {x, y};
                }
            if (cursor[1]) printf("\e[%uA", cursor[1]);
            fflush(stdout);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
