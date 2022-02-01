#ifndef kalmanLibrary_h_   
#define kalmanLibrary_h_
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

// function to formulate the state transition matrix of a differential drive robot 
void state_transition_matrix_diff_drive(float delta_t, float wheel_base, gsl_vector * state, gsl_vector * control, gsl_matrix * state_transition);

// function to formulate the control matrix for a differential drive robot
void control_matrix_diff_drive(float delta_t, float wheel_base, gsl_vector * state, gsl_vector * control, gsl_matrix * control_matrix);

// function to formulate the process uncertainity of a differential drive robot
void process_uncertainity_diff_drive(float variance[], gsl_vector * control, gsl_matrix * control_matrix ,gsl_matrix * process_uncertainity);

// function to formulate the observation matrix to reshape to states to match measurement for a differential drive robot
void observation_matrix_diff_drive(float landmark_positions[], gsl_vector * state, gsl_matrix * observation_matrix);

// function to formulate the meaurement uncertainity matrix for a differential drive robot 
void measurement_uncertainity_diff_drive(float variance[], gsl_matrix * measurement_uncertainity);

#endif
