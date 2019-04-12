#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  // Kp = Kp_;
  // Ki = Ki_;
  // Kd = Kd_;

  K.push_back(Kp_);
  K.push_back(Ki_);
  K.push_back(Kd_);

  

  // dp_Kp = 0.2;
  // dp_Ki = 0.2;
  // dp_Kd = 0.2;

  dp_K.push_back(0.2);
  dp_K.push_back(0.2);
  dp_K.push_back(0.2);

  iter = 0;
  started = false;
  best_error = std::numeric_limits<double>::infinity();
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  if (started == false) {
    p_error = cte;
    i_error = 0.0;
    started = true;
  }

  i_error += cte;
  d_error = cte - p_error;
  p_error = cte;
  
  // iter++;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return - K[0] * p_error - K[2] * d_error - K[1] * i_error;  // TODO: Add your total error calc here!
}

void PID::Twiddle() {

  if(iter > 200) {
    double err = 0.0;
    double tol = 0.00001;
    double params = dp_K[0] + dp_K[1] + dp_K[2];
    std::cout << "\nkp: " << K[0] << ",    ki: " << K[1] << ",    kp: " << K[2] << std::endl;
    std::cout << "dp_kp: " << dp_K[0] << ", dp_ki: " << dp_K[1] << ", dp_kp: " << dp_K[2] << std::endl;
    std::cout << "PARAMS: " << params << std::endl;
    if (params > tol) {

      for (unsigned int i = 0; i < K.size(); i++) {
        K[i] = K[i] += dp_K[i];
        err = abs(TotalError());

        if (err < best_error) {
          best_error = err;
          dp_K[i] *= 1.1;

        } else {
          K[i] = K[i] -= 2.0 * dp_K[i];
          err = abs(TotalError());

          if (err < best_error) {
            best_error = err;
            dp_K[i] *= 1.1;

          } else {
            K[i] = K[i] += dp_K[i];
            dp_K[i] *= 0.9;
          }
        }
      }
    }
  }
  iter++;
}
