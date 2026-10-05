# pragma once

#include <array>
#include <cstddef>
#include <algorithm>


class Controller
{
    public:
        std::array<double, 6> p_controller(const std::array<double, 6>& current,
                                               const std::array<double, 6>& target);
};
