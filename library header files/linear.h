#ifndef kalmanLibrary_h_   
#define kalmanLibrary_h_
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

// function to formulate the state transition matrix to estimate position and velocity for a linear system 
void state_transition_linear_pos_vel(float delta_t, gsl_matrix * state_transition_matrix);

// function to formulate the control matrix for a linear system with acceleration as input
void control_matrix_linear_acc(float delta_t, gsl_matrix * control_matrix);

// function to formulate the process uncertainity a linear system with acceleration as input
void process_uncertainity_linear_acc(float variance, gsl_matrix * control_matrix, gsl_matrix * process_uncertainity);

// function to formulate the observation matrix to reshape to states to match measurement for a linear system 
void observation_matrix_linear_pos_vel(gsl_matrix * observation_matrix);

// function to formulate the meaurement uncertainity matrix for the measured position and velocity in a linear system 
void measurement_uncertainity_linear_pos_vel(const float Variance[], gsl_matrix * measurement_uncertainity);

#endif