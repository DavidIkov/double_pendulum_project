
#include <stdio.h>

#include <chrono>
#include <thread>
#include <vector>

#include "pendulum/double.hpp"

int main(int, char**) {
    DoublePendulum pend({.angle_ = 0}, {.angle_ = 0});
    float radius = pend.GetMaxRadius();

    mathcpp::Vector2U boxes_amount{40, 20};

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

    float counter = 0.f;
    while (1) {
        std::fill(boxes_back.begin(), boxes_back.end(), 0);

        mathcpp::Vector2F p0 = pend.GetPendulum0Pos() + radius;
        mathcpp::Vector2F p1 = pend.GetPendulum1Pos() + radius;

        mathcpp::Vector2U p0_box(p0 / box_size);
        mathcpp::Vector2U p1_box(p1 / box_size);

        counter += 0.1f;
        pend = DoublePendulum({.angle_ = counter}, {.angle_ = -counter / 2});

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

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
