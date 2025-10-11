
#include "renderer.hpp"

#include <stdexcept>

ConsoleRenderer::ConsoleRenderer(mathcpp::Vector2U boxes_amount)
    : size_(boxes_amount),
      front_(boxes_amount[0] * boxes_amount[1], 0),
      back_(front_.size(), 0) {
    {  // PREPARE CONSOLE
        printf("TOP\n");
        for (size_t i = 0; i < boxes_amount[1]; ++i) printf("\n");
        printf("BOTTOM");
        printf("\r\e[%uA", (unsigned)boxes_amount[1]);
        fflush(stdout);
    }
}

void ConsoleRenderer::ClearBackBuffer() {
    std::fill(back_.begin(), back_.end(), 0);
}
void ConsoleRenderer::SwapBuffers() {
    mathcpp::Vector2U cursor{0, 0};
    for (size_t i = 0; i < front_.size(); ++i)
        if (back_[i] != front_[i]) {
            unsigned x = i % size_[0], y = i / size_[0];
            if (unsigned off = y - cursor[1]) printf("\e[%uB", off);
            if (x) printf("\e[%uC", x);
            if (back_[i])
                printf("+");
            else
                printf(" ");
            printf("\r");
            cursor = {x, y};
        }
    if (cursor[1]) printf("\e[%uA", cursor[1]);
    fflush(stdout);

    front_.swap(back_);
}

mathcpp::Vector2U ConsoleRenderer::GetSize() const { return size_; }

void ConsoleRenderer::Draw(mathcpp::Vector2U pos) {
    if (pos[0] >= size_[0] || pos[1] >= size_[1])
        throw std::runtime_error("incorrect position");
    back_[pos[1] * size_[0] + pos[0]] = 1;
}

void ConsoleRenderer::Draw(mathcpp::Vector2F pos) {
    if (pos[0] < -1 || pos[0] > 1 || pos[1] < -1 || pos[1] > 1)
        throw std::runtime_error("incorrect position");

    pos = {pos[0] + 1, 1 - pos[1]};

    mathcpp::Vector2F box_size = mathcpp::Vector2F(2.f) / size_;
    mathcpp::Vector2U pos_in_box(pos / box_size);

    Draw(pos_in_box.Clamp({0, 0}, {size_[0] - 1, size_[1] - 1}));
}
