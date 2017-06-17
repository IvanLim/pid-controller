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

  // PID Coefficients
  double Kp;
  double Ki;
  double Kd;

  // Twiddle tuning parameters
  double dp;
  double di;
  double dd;

  // Twiddle variables we need to track
  long twiddle_n;                 // The length of a twiddle run
  double twiddle_cumulative_err;  // The cumulative error of a twiddle run
  double twiddle_best_err;        // The twiddle best error so far for a given coefficient/parameter

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

  /*
  * Performs an update of the Twiddle parameters, used to tune our PID coefficients
  */
  void TwiddleUpdate();

private:
  /* 
    Returns a pointer to the current twiddle coefficient we're tuning
  */
  double *GetCurrentTwiddleCoefficient();

  /* 
    Returns a pointer to the current twiddle tuning parameter
  */
  double *GetCurrentTwiddleTuningParameter();

  /* 
    Moves on to the next Twiddle coefficient
  */
  void MoveToNextTwiddleCoefficient();

};

#endif /* PID_H */
