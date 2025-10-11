#pragma once

#include <vector>

#include "mathcpp/vector.hpp"

class ConsoleRenderer {
public:
    ConsoleRenderer(mathcpp::Vector2U boxes_amount);

    void ClearBackBuffer();
    void SwapBuffers();

    // starts from top left, measured in boxes
    void Draw(mathcpp::Vector2U pos);
    // from (-1,-1) is bottom left, (1,1) is bottom right
    void Draw(mathcpp::Vector2F pos);

    mathcpp::Vector2U GetSize() const;

private:
    mathcpp::Vector2U size_;
    std::vector<bool> front_;
    std::vector<bool> back_;
};
