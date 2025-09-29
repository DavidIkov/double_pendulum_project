
#include <stdio.h>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "pendulum/double.hpp"

int main(int, char**) {
    DoublePendulum pend({.angle_ = M_PI, .length_ = 100.f, .mass_ = 3.f},
                        {.angle_ = M_PI / 2, .length_ = 100.f, .mass_ = 3.f});
    float radius = pend.GetMaxRadius();

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

        mathcpp::Vector2F p0 = pend.GetPendulum0Pos();
        mathcpp::Vector2F p1 = pend.GetPendulum1Pos();

        p0 = {p0[0] + radius, radius - p0[1]},
        p1 = {p1[0] + radius, radius - p1[1]};

        mathcpp::Vector2U p0_box(p0 / box_size);
        mathcpp::Vector2U p1_box(p1 / box_size);

        for (size_t step = 0; step < 100; ++step) pend.Step(0.001f);

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
