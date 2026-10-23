# pragma once

#include <array>
#include <cstddef>
#include <algorithm>

#include "state_manager.h"

class Controller
{
    public:
        void set_gains(const PGains& p_gains);
        std::array<double, 6> p_controller(const std::array<double, 6>& current,
                                               const std::array<double, 6>& target);
    private:
            PGains p_gains_;
};
