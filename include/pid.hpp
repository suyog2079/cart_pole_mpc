#include <algorithm>

class PID {
private:
  double kp, ki, kd;
  double dt;

  double integral = 0.0;
  double previous_error = 0.0;
  double previous_measurement = 0.0;

  double integral_limit;
  double output_limit;

public:
  PID(double kp, double ki, double kd, double dt, double integral_limit,
      double output_limit)
      : kp(kp), ki(ki), kd(kd), dt(dt), integral_limit(integral_limit),
        output_limit(output_limit) {}

  double calculate(double setpoint, double measured_value) {
    double error = setpoint - measured_value;
    integral += error * dt;
    integral = std::clamp(integral, -integral_limit, integral_limit);
    double derivative = -(measured_value - previous_measurement) / dt;
    double output = kp * error + ki * integral + kd * derivative;
    output = std::clamp(output, -output_limit, output_limit);
    previous_error = error;
    previous_measurement = measured_value;
    return output;
  }
};
