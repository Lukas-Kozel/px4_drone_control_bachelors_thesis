#ifndef PID_H_
#define PID_H_

#include <cstdint>
#include <algorithm> // for std::max

enum PID_MODE {
	PID_MODE_DERIVATIV_NONE = 0,
	PID_MODE_DERIVATIV_CALC,
	PID_MODE_DERIVATIV_CALC_NO_SP,
	PID_MODE_DERIVATIV_SET
};

struct PID_t {
	PID_MODE mode;
	float dt_min;
	float kp;
	float ki;
	float kd;
	float integral;
	float integral_limit;
	float output_limit;
	float error_previous;
	float last_output;
};
	void pid_init(PID_t *pid, PID_MODE mode, float dt_min);
	int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
	float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt);


#endif /* PID_H_ */
