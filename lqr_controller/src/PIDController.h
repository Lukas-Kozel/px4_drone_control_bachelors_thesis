#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double integral_max, double integral_min);

    double compute(double error, double dt);

private:
    double kp_;
    double ki_;
    double kd_;
    double integral_max_;
    double integral_min_;
    double integral_;
    double previous_error_;
};

#endif // PIDCONTROLLER_H
