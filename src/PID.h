#ifndef PID_H
#define PID_H

class PID {
public:

  // Enums for our state machine
  enum class Coefficient {
    coefficient_Kp = 0,
    coefficient_Ki = 1,
    coefficient_Kd = 2
  };

  enum class TwiddleState {
    stage_one = 0,
    stage_two = 1,
    stage_three = 2
  };

  // Keeps track of the current twiddle state
  TwiddleState twiddle_state;

  // Keeps track of the current PID coefficient that we're tuning
  Coefficient current_coefficient;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double dp;
  double di;
  double dd;

  int twiddle_cycle_count;
  
  long twiddle_n;
  double twiddle_cumulative_err;
  double twiddle_best_err;

  double *current_twiddle_coefficient;
  double *current_twiddle_tuning_parameter;



  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void TwiddleUpdate();

private:
  double *GetCurrentTwiddleCoefficient();
  double *GetCurrentTwiddleTuningParameter();
  void MoveToNextTwiddleCoefficient();

};

#endif /* PID_H */
