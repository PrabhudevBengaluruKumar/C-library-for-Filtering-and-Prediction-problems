#ifndef kalmanLibrary_h_   
#define kalmanLibrary_h_
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
// function to estimate the state variables
void predict_state(const gsl_matrix * state_transition_matrix, gsl_vector * updated_state, const gsl_matrix * control_matrix, const gsl_vector * control, gsl_vector * predicted_state);
// function to estimate the uncertainity in predictions
void predict_covariance(const gsl_matrix * state_transition_matrix, gsl_matrix * covariance_matrix, const gsl_matrix * process_uncertainity, gsl_matrix * predicted_covariance);
// function to compute kalman gain
void compute_kalman_gain(gsl_matrix * predicted_covariance, gsl_matrix * observation_matrix, gsl_matrix * measurement_uncertainity, gsl_matrix * kalman_gain);
// function to update the state
void update_state(gsl_vector * predicted_state, gsl_matrix * kalman_gain, gsl_matrix * observation_matrix, gsl_vector * measurement_vector, gsl_vector * updated_state);
// function to update the covariance
void update_covariance(gsl_matrix * predicted_covariance, gsl_matrix * kalman_gain, gsl_matrix * observation_matrix,gsl_matrix * Identity_matrix, gsl_matrix * updated_covariance);
#endif
