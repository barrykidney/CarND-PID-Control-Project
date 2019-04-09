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
  Kp = 0.2;
  Ki = 0.004;
  Kd = 3.0;
  started = false;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = 0;
  i_error = 0;
  d_error = 0;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  

  return 0.0;  // TODO: Add your total error calc here!
}

double PID::GetKp() {
  return Kp;
}

double PID::GetKi() {
  return Ki;
}

double PID::GetKd() {
  return Kd;
}

double PID::GetPrev_cte() {
  return prev_cte;
}

double PID::GetSum_cte() {
  return sum_cte;
}

bool PID::GetStarted() {
  return started;
}

void PID::SetPrev_cte(double cte) {
  prev_cte = cte;
}

void PID::SetSum_cte(double cte) {
  sum_cte = cte;
}

void PID::SetStarted(bool s) {
  started = s;
}