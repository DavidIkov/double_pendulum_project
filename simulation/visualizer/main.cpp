
#include <vector>

#include "pendulum/double.hpp"

int main(int, char**) {
    DoublePendulum pend({}, {});
    float radius = pend.GetMaxRadius();

    size_t boxes_amount_on_axis = 20;

    float box_size = radius * 2 / boxes_amount_on_axis;

    std::vector<bool> boxes_front(boxes_amount_on_axis * boxes_amount_on_axis);
    std::vector<bool> boxes_back = boxes_front;
    

    return 0;
}
