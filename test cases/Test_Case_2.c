//Calculating the position and velocity of a body in 2-Dimension
#include<stdio.h>
#include "linear.h"
#include "kalmanLibrary.h"
void main()
{
    int i, j;
    printf("\n------------------------- SAMPLE CASE 2 ------------------------");
    printf("\n Estimating the position and velocity of a body in 2-Dimension ");
    printf("\n----------------------------------------------------------------\n");
     
    float *state_array=malloc(4 * sizeof(float));
    //initial position and initial velocity 
    state_array[0] = 10;
    state_array[1] = 5;
    state_array[3] = 0.75;
    state_array[4] = 0.9;
    
    // allocating memory and initializing state variables
    gsl_vector * state = gsl_vector_alloc (4);
    for (i = 0; i < 4; i++)
    {
      gsl_vector_set (state, i, state_array[i]);
    } 
    
    printf("\n The initial position and velocity at which the body starts is:\n");
    for (i = 0; i < 4; i++)
    {
      printf ("%g\n", gsl_vector_get (state, i));
      printf("\n");
    }
    free(state_array);
    printf("\n----------------------------------------------------------------\n");
    float *control_array=malloc(2 * sizeof(float));
    
    //acceleration at which the body is moving
    control_array[0] = 2.5;
    control_array[1] = 3.1;
    // allocating memory for control variables
    gsl_vector * control = gsl_vector_alloc (2);
    for (i = 0; i < 2; i++)
    {
        gsl_vector_set (control, i, control_array[i]);
    }
    printf("\n The acceleration at which the body is moving is: \n");
    for (i = 0; i < 2; i++)
    {
      printf ("%g\n", gsl_vector_get (control, i));
    }
    free(control_array);
    printf("\n----------------------------------------------------------------\n"); 
    float delta_t = 0.5;
    printf("\n The time interval at which the measurements are made is: %f", delta_t);
    printf("\n----------------------------------------------------------------\n");
    // formulating state transition matrix
    gsl_matrix * state_transition_matrix = gsl_matrix_calloc (4, 4);
    state_transition_linear_pos_vel(delta_t, state_transition_matrix);
   // formulating control matrix
    gsl_matrix * control_matrix = gsl_matrix_calloc (4, 2);
    control_matrix_linear_acc(delta_t, control_matrix);
   // allocating memory for uncertainity matrix
    gsl_matrix * covariance_matrix = gsl_matrix_calloc (4, 4);
    gsl_matrix_set_identity(covariance_matrix);
    // predict state
    gsl_vector * predicted_state = gsl_vector_calloc (4);
    predict_state(state_transition_matrix, state, control_matrix, control, predicted_state);
    // formulating process uncertainity matrix
    float Variance_acceleration = 0.75;
    gsl_matrix * process_uncertainity  = gsl_matrix_calloc (4, 4);
    process_uncertainity_linear_acc(Variance_acceleration, control_matrix, process_uncertainity);
    // predict covariance
    gsl_matrix * predicted_covariance= gsl_matrix_calloc (4, 4);
    predict_covariance(state_transition_matrix, covariance_matrix, process_uncertainity, predicted_covariance);
    // formulate observation_matrix
    gsl_matrix * observation_matrix= gsl_matrix_calloc (2, 4);
    observation_matrix_linear_pos_vel(observation_matrix);
    // formulate measurement_uncertainity
    gsl_matrix * measurement_uncertainity= gsl_matrix_calloc (2, 2);
    float variance_measurement[] = {0.11, 0.23};
    measurement_uncertainity_linear_pos_vel(variance_measurement, measurement_uncertainity);
    gsl_matrix * kalman_gain = gsl_matrix_calloc (4, 2);
    gsl_vector * updated_state = gsl_vector_calloc (4);
    gsl_matrix * updated_covariance = gsl_matrix_calloc (4, 4);
    gsl_matrix * Identity_matrix = gsl_matrix_calloc (4, 4);
    gsl_matrix_set_identity(Identity_matrix);
    //initializing the list of measurements
    float measurement_array[10][2] = {{10.11, 5.99}, {12.16,7.12}, {12.9,8}, {15.67,9.5}, {16.2,11.33}, {17.53,12.5}, {19.1,12.9}, {20,14.02}, {21.12, 14.89}, {22.25,15.76}};
    printf("\n Recorded measurements are:\n");
    for (i = 0; i < 10; i++)
    {
        for(j=0; j<2; j++){
            printf("%.3f \t",measurement_array[i][j]);
        }
        printf("\n");
    }
    printf("\n----------------------------------------------------------------\n");
    for(i=0; i<10; i++)
    {
        gsl_vector * measurement_vector = gsl_vector_alloc (2);
        for(j=0; j<2; j++){
            gsl_vector_set (measurement_vector, j, measurement_array[i][j]);
        }
        // calculate kalman gain
        compute_kalman_gain(predicted_covariance, observation_matrix, measurement_uncertainity, kalman_gain);
        
        // update state
        update_state(predicted_state, kalman_gain, observation_matrix, measurement_vector, updated_state);
        // update covariance
        update_covariance(predicted_covariance, kalman_gain, observation_matrix, Identity_matrix, updated_covariance);
        gsl_matrix_set_identity(Identity_matrix); 
        // predict state
        predict_state(state_transition_matrix, updated_state, control_matrix, control, predicted_state);
        // predict covariance
        predict_covariance(state_transition_matrix, updated_covariance, process_uncertainity, predicted_covariance);
        gsl_vector_free (measurement_vector);
   
    }
    printf("\n Predicted state:\n");
    for (i = 0; i < 4; i++)
    {
      printf ("%g \t", gsl_vector_get (predicted_state, i));
      printf("\n");
    }
    printf("\n Predicted covariance:\n");
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            printf ("%g \t", gsl_matrix_get (predicted_covariance, i, j));
        }
        printf("\n");
    } 
}
