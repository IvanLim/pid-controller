#include <iostream>
#include <cmath>
#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    // These are the tuning parameter values obtained after
    // going through the twiddle tuning process
    dp = -0.147789;
    di = 0.00106112;
    dd = 2.95378;

    // Initialize twiddle values
    twiddle_n = 0;
    twiddle_cumulative_err = 0;
    twiddle_best_err = 0;

    // Initialize the state machine
    twiddle_state = TwiddleState::stage_one;
    current_coefficient = Coefficient::coefficient_Kp;
}

void PID::UpdateError(double cte) {

    // Keep track of the twiddle error
    twiddle_cumulative_err += cte * cte;

    // D error = cte - previous_cte
    d_error = cte - p_error;

    // P error = cte
    p_error = cte;

    // I error = cumulative cte
    i_error += cte;
}

// Main twiddle logic
// The twiddle process here is organized here as a state machine
// Each stage is commented in detail below
void PID::TwiddleUpdate() {
    // Get the pointers of the coefficient and parameter we want to work with
    current_twiddle_coefficient = GetCurrentTwiddleCoefficient();
    current_twiddle_tuning_parameter = GetCurrentTwiddleTuningParameter();

    // Calculate the error of the last run
    double twiddle_err = twiddle_cumulative_err / twiddle_n;

    switch(twiddle_state) {
        case TwiddleState::stage_one:
            // This is a fresh run. Use the current err as the best_err
            twiddle_best_err = twiddle_err;

            // ADD tuning parameter to our coefficient
            *current_twiddle_coefficient += *current_twiddle_tuning_parameter;

            // Move on to next state, and wait for results
            twiddle_state = TwiddleState::stage_two;
            break;

        case TwiddleState::stage_two:
            // If the results are better than what we have
            // Keep the results, increase our tuning parameter
            // and move to the next twiddle coefficient
            if (twiddle_err < twiddle_best_err) {
                twiddle_best_err = twiddle_err;
                *current_twiddle_tuning_parameter *= 1.1;
                MoveToNextTwiddleCoefficient();
            } else {
                // Our previous results were better. 
                // Undo our most recent change, and
                // try the opposite direction (MINUS the tuning parameter)
                *current_twiddle_coefficient -= 2 * *current_twiddle_tuning_parameter;

                // Move on to next state and wait for results
                twiddle_state = TwiddleState::stage_three;
            }            
            break;

        case TwiddleState::stage_three:
            // if stage_three:
            //   if cte < best_error:
            //     best_error = cte;
            //     current_modifier *= 1.1
            //   else:
            //     current_coefficient += current modifier
            //     current_modifier *= 0.9
            //   move to next coefficient
            //   move back to first train state stage_one

            // If the results are better than what we have
            // Keep the results, increase our tuning parameter
            if (twiddle_err < twiddle_best_err) {
                twiddle_best_err = twiddle_err;
                *current_twiddle_tuning_parameter *= 1.1;
            } else {
                // Our previous results were better, even after trying both directions.
                // Undo our changes
                *current_twiddle_coefficient += *current_twiddle_tuning_parameter;

                // Reduce the tunning parameter for our next try
                *current_twiddle_tuning_parameter *= 0.9;
            }

            // Move on to the next coefficient and start again
            MoveToNextTwiddleCoefficient();
            break;

        default: break;
    }

}

// Move to the next twiddle coefficient, and reset the twiddle state
void PID::MoveToNextTwiddleCoefficient() {    
    current_coefficient = static_cast<Coefficient>((static_cast<int>(current_coefficient) + 1) % 3);        
    twiddle_state = TwiddleState::stage_one;
}

// Returns a pointer to the current coefficient we're twiddling
double *PID::GetCurrentTwiddleCoefficient() {
    switch(current_coefficient) {
        case Coefficient::coefficient_Kp: return &(this->Kp); break;
        case Coefficient::coefficient_Ki: return &(this->Ki); break;
        case Coefficient::coefficient_Kd: return &(this->Kd); break;
        default: return nullptr; break;
    }
}

// Returns a pointer to the current tuning parameter
double *PID::GetCurrentTwiddleTuningParameter() {
    switch(current_coefficient) {
        case Coefficient::coefficient_Kp: return &dp; break;
        case Coefficient::coefficient_Ki: return &di; break;
        case Coefficient::coefficient_Kd: return &dd; break;
        default: return nullptr; break;
    }
}

// Returns the output of the PID controller
double PID::TotalError() {
    return (Kp * p_error) + (Kd * d_error) + (Ki * i_error);
}