#ifndef PID_H
#define PID_H

#include <vector>
#include <limits>

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

  void Twiddle();

  std::vector<double> K;
  std::vector<double> dp_K;

  double GetKp();
  double GetKi();
  double GetKd();

  double GetDp_Kp();
  double GetDp_Ki();
  double GetDp_Kd();

  void SetKp(double Kp_);
  void SetKi(double Ki_);
  void SetKd(double Kd_);

  void SetDp_Kp(double dp_Kp_);
  void SetDp_Ki(double dp_Ki_);
  void SetDp_Kd(double dp_Kd_);
  
  // std::vector<double> Twiddle(std::vector<double> params);

  double best_error;
  bool twiddle;
  int iter;
  int p_iter;

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
  double Kp;
  double Ki;
  double Kd;

  double dp_Kp;
  double dp_Ki;
  double dp_Kd;

  bool started;
  
};

#endif  // PID_H