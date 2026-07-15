#ifndef REAL_PID_H
#define REAL_PID_H

namespace core_move
{
    class RealPID
    {
    private:
        double err_now_ = 0;
        double err_before_ = 0;
        double err_before_before_ = 0;

        double result_now_ = 0;
        double result_before_ = 0;

        double output_before = 0;

    public:
        double para_p = 0;
        double para_i = 0;
        double para_d = 0;
        double speed_limit_max = 0;
        double speed_limit_min = 0;
        double accelerate_max = 0;
        double para_rate = 0;

        RealPID(double Kp = 0, double Ki = 0, double Kd = 0, double speed_max = 0, double speed_min = 0, double acc_max = 0, double rate = 0);
        void clearIntegrator();
        double calOutput(double err);
    };
}

#endif
