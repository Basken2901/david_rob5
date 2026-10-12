#include "controller.h"

void Controller::set_gains(
    const PGains& p_gains
)
{
    p_gains_ = p_gains;
}

std::array<double, 6> Controller::p_controller(const std::array<double, 6>& current,
                                               const std::array<double, 6>& target)
{
    const double kp = p_gains_.Kp;
    const double max_vel = 0.5;  // rad/s, safety limit

    std::array<double, 6> vel{};
    for (size_t i = 0; i < 6; ++i)
    {
        double error = target[i] - current[i];
        vel[i] = std::clamp(kp * error, -max_vel, max_vel);
    }
    return vel;
}
