#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  double diff_cte = cte - p_error;
  p_error = cte;
  d_error = diff_cte;
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total_error = -Kp*p_error - Kd*d_error - Ki*i_error;
  if (total_error > 1) {
    total_error = 1;
  }
  if (total_error < -1) {
    total_error = -1;
  }
  return total_error;  // TODO: Add your total error calc here!
}