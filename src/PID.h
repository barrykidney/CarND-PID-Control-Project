#ifndef PID_H
#define PID_H

#include <vector>
#include <limits>
#include <cmath>
#include <iostream>
#include <iomanip>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void Twiddle(double cte);

  std::vector<double> K;
  std::vector<double> dp_K;

  double best_error;
  double total_error;
  double tol;
  bool started;
  bool first;
  bool add_dp_k;
  int ki;
  int iter;

//  private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  // double Kp;
  // double Ki;
  // double Kd;
};

#endif  // PID_H