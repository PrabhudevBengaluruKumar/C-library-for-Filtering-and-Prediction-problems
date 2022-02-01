#include <stdio.h>
#include <math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "kalmanLibrary.h"

// function to estimate the state variables

void predict_state(const gsl_matrix * state_transition_matrix, gsl_vector * updated_state, const gsl_matrix * control_matrix, const gsl_vector * control, gsl_vector * predicted_state)
{
	
    gsl_blas_dgemv(CblasNoTrans, 1.0, state_transition_matrix, updated_state, 0.0, predicted_state);
    gsl_blas_dgemv(CblasNoTrans, 1.0, control_matrix, control, 1.0, predicted_state);

}

// function to estimate the uncertainity in predictions
void predict_covariance(const gsl_matrix * state_transition_matrix, gsl_matrix * covariance_matrix, const gsl_matrix * process_uncertainity, gsl_matrix * predicted_covariance)
{
 
    int m = state_transition_matrix->size1;
    gsl_matrix * F_Ft = gsl_matrix_calloc (m, m);
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1, state_transition_matrix, state_transition_matrix, 0.0, F_Ft);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, F_Ft, covariance_matrix, 0.0, predicted_covariance);
    gsl_matrix_add(predicted_covariance, process_uncertainity);
 
}

// function to compute kalman gain
void compute_kalman_gain(gsl_matrix * predicted_covariance, gsl_matrix * observation_matrix, gsl_matrix * measurement_uncertainity, gsl_matrix * kalman_gain)
{
	
    int i, j, signum = 0;
    int m = observation_matrix->size1;
    int n = observation_matrix->size2;
    gsl_matrix * H_P_Ht = gsl_matrix_calloc (m, m);
    gsl_matrix * P_Ht = gsl_matrix_calloc (n, m);
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1, predicted_covariance, observation_matrix, 0.0, P_Ht); 
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, observation_matrix, P_Ht, 0.0, H_P_Ht);
    gsl_matrix_add(H_P_Ht, measurement_uncertainity);
    gsl_linalg_cholesky_decomp(H_P_Ht);
    gsl_linalg_cholesky_invert(H_P_Ht);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, P_Ht, H_P_Ht, 0.0, kalman_gain);

}

// function to update the state
void update_state(gsl_vector * predicted_state, gsl_matrix * kalman_gain, gsl_matrix * observation_matrix, gsl_vector * measurement_vector, gsl_vector * updated_state)
{
	
    int m = observation_matrix->size1;
    int n = observation_matrix->size2;
   
    gsl_vector * Hx = gsl_vector_calloc (m);
    gsl_blas_dgemv(CblasNoTrans, 1.0, observation_matrix, predicted_state, 0.0, Hx);
    gsl_vector_sub(measurement_vector, Hx);
    gsl_blas_dgemv(CblasNoTrans, 1.0, kalman_gain, measurement_vector, 1.0, predicted_state);
    gsl_vector_memcpy(updated_state, predicted_state);
    
}

// function to update the covariance
void update_covariance(gsl_matrix * predicted_covariance, gsl_matrix * kalman_gain, gsl_matrix * observation_matrix, gsl_matrix * Identity_matrix, gsl_matrix * updated_covariance)
{
	
    int n = observation_matrix->size2;
    gsl_matrix * KH = gsl_matrix_calloc (n, n);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, kalman_gain, observation_matrix, 0.0, KH);
    gsl_matrix_sub(Identity_matrix, KH);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, Identity_matrix, predicted_covariance, 0.0, updated_covariance);

}





