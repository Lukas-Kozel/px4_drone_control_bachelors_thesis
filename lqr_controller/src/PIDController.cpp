#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd, double integral_max, double integral_min)
    : kp_(kp), ki_(ki), kd_(kd), integral_max_(integral_max), integral_min_(integral_min), integral_(0.0), previous_error_(0.0) {}

double PIDController::compute(double error, double dt) {
    integral_ += error * dt;
    
    // Implementing integral windup protection (saturation)
    if (integral_ > integral_max_) {
        integral_ = integral_max_;
    } else if (integral_ < integral_min_) {
        integral_ = integral_min_;
    }

    double derivative = (error - previous_error_) / dt;
    previous_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}
