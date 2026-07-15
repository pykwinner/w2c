#include "upros_core_move/real_pid.h"
#include "algorithm"
#include "ros/ros.h"

namespace core_move
{

    RealPID::RealPID(double Kp, double Ki, double Kd, double speed_max, double speed_min, double acc_max, double rate)
    {
        para_p = Kp;
        para_i = Ki;
        para_d = Kd;
        speed_limit_max = speed_max;
        speed_limit_min = speed_min;
        accelerate_max = acc_max;
        para_rate = rate;
    }

    void RealPID::clearIntegrator()
    {
        err_now_ = 0;
        err_before_ = 0;
        err_before_before_ = 0;
        result_now_ = 0;
        result_before_ = 0;
        output_before = 0;
    }

    double RealPID::calOutput(double err)
    {
        double vel_output;

        err_before_before_ = err_before_;
        err_before_ = err_now_;
        err_now_ = err;
        result_before_ = result_now_;

        result_now_ = para_p * (err_now_ - err_before_) + para_i / para_rate * err_now_ +
                      para_d * (err_now_ - 2 * err_before_ + err_before_before_) + result_before_;

        if (result_now_ > 0)
        {
            vel_output = std::max(std::min(result_now_, speed_limit_max), speed_limit_min);
        }
        else
        {
            vel_output = std::min(std::max(result_now_, -1 * speed_limit_max), -1 * speed_limit_min);
        }

        vel_output = std::max(std::min(vel_output, output_before + accelerate_max / para_rate),
                              output_before - accelerate_max / para_rate);

        output_before = vel_output;
        return vel_output;
    }

}
