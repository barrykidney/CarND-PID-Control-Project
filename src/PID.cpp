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

  K.push_back(Kp_);
  K.push_back(Ki_);
  K.push_back(Kd_);

  dp_K.push_back(0.022);
  dp_K.push_back(0.00045);
  dp_K.push_back(0.018);

  tol = 0.0001;
  ki = 0;
  iter = 0;
  started = false;
  total_error = 0.0;
  first = true;
  add_dp_k = true;

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
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  // return - (p_error * 0.25) - (d_error * 25) - (i_error * 0.003);
  // return - (p_error * 0.4) - (d_error * 12) - (i_error * 0.003);
  return - (K[0] * p_error - K[2] * d_error - K[1] * i_error);  // TODO: Add your total error calc here!
}

void PID::Twiddle(double cte) {

  if ((dp_K[0] + dp_K[1] + dp_K[2]) > tol) { 
  std::cout << std::setprecision(9) << std::fixed;

  if (iter % 2200 > 200) {
     total_error += pow(cte, 2);
  }

  if (iter > 2200 && iter % 2200 == 200) {

    std::cout << "\nTWIDDLE,  PARAMETER " << (ki % 3) + 1<< std::endl;

    if (add_dp_k == true) {
      K[ki] = K[ki] += dp_K[ki];
    } else {
      K[ki] = K[ki] -= dp_K[ki];
    }

    std::cout << "kp: " << K[0] << ",    ki: " << K[1] << ",    kp: " << K[2] << std::endl;
    std::cout << "dp_kp: " << dp_K[0] << ", dp_ki: " << dp_K[1] << ", dp_kp: " << dp_K[2] << std::endl;

  } else if (iter != 0 && iter % 2200 == 0) {
    if (first == false) {
      if (add_dp_k == true && (total_error  / 2000) < best_error) {
        best_error = total_error / 2000;
        dp_K[ki] *= 1.1;
        std::cout << "IMPROVEMENT WITH +, (" << dp_K[ki] << "), BEST ERROR: " << best_error << std::endl;
        ki++;
      } else if (add_dp_k == true && (total_error  / 2000) > best_error) {
        K[ki] = K[ki] -= dp_K[ki];
        add_dp_k = false;
        std::cout << "NO IMPROVEMENT TRY -" << std::endl;
      } else if (add_dp_k == false && (total_error  / 2000) < best_error) {
        best_error = total_error / 2000;
        dp_K[ki] *= 1.1;
        std::cout << "IMPROVEMENT WITH -, (" << dp_K[ki] << "), BEST ERROR: " << best_error << std::endl;
        add_dp_k = true;
        ki++;
      } else if (add_dp_k == false && (total_error  / 2000) > best_error) {
        K[ki] = K[ki] += dp_K[ki];
        dp_K[ki] *= 0.9;
        std::cout << "NO IMPROVEMENT(" << dp_K[ki] << ")" << std::endl;
        add_dp_k = true;
        ki++;
      }
    } else {
      best_error = total_error / 2000;
      first = false;
      std::cout << "FIRST LAP, BEST ERROR: " << best_error << std::endl;
    }
    total_error = 0.0;
  }
  iter++;
  } else {
    std::cout << "FINAL RESULT, kp: " << K[0] << ",ki: " << K[1] << ",kp: " << K[2] << std::endl;
  }
}
