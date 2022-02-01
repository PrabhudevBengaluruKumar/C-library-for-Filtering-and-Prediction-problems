//Calculating the position and velocity of a body in 1-Dimension

#include<stdio.h>
#include "linear.h"
#include "kalmanLibrary.h"

void main()
{
    int i, j;

    printf("\n------------------------- SAMPLE CASE 1 ------------------------");
    printf("\n Estimating the position and velocity of a body in 1-Dimension ");
    printf("\n----------------------------------------------------------------\n");

     
    float *state_array=malloc(2 * sizeof(float));

    //initial position and initial velocity 
    state_array[0] = 10;
    state_array[1] = 0.75;
    
    // allocating memory and initializing state variables
    gsl_vector * state = gsl_vector_alloc (2); 
    for (i = 0; i < 2; i++)
    {
      gsl_vector_set (state, i, state_array[i]);
    }
    
    printf("\n The initial position and velocity at which the body starts is:\n");
    for (i = 0; i < 2; i++)
    {
      printf ("%g\t", gsl_vector_get (state, i));
      printf("\n");
    }
    free(state_array);

    printf("\n----------------------------------------------------------------\n");

    float *control_array=malloc(1 * sizeof(float));
    
    //acceleration at which the body is moving
    control_array[0] = 2.5;
    
    // allocating memory for control variables
    gsl_vector * control = gsl_vector_alloc (1);
    gsl_vector_set (control, 0, control_array[0]);
    printf("\n The acceleration at which the body is moving is: %g", gsl_vector_get (control, 0));
    free(control_array);

    printf("\n----------------------------------------------------------------\n"); 

    float delta_t = 0.5;
    printf("\n The time interval at which the measurements are made is: %f", delta_t);

    printf("\n----------------------------------------------------------------\n");

    // formulating state transition matrix
    gsl_matrix * state_transition_matrix = gsl_matrix_calloc (2, 2);
    state_transition_linear_pos_vel(delta_t, state_transition_matrix);

   // formulating control matrix
    gsl_matrix * control_matrix = gsl_matrix_calloc (2, 1);
    control_matrix_linear_acc(delta_t, control_matrix);

   // allocating memory for uncertainity matrix
    gsl_matrix * covariance_matrix = gsl_matrix_calloc (2, 2);
    gsl_matrix_set_identity(covariance_matrix);

    // predict state
    gsl_vector * predicted_state = gsl_vector_calloc (2);
    predict_state(state_transition_matrix, state, control_matrix, control, predicted_state);

    // formulating process noise matrix
    float Variance_acceleration = 0.75;
    gsl_matrix * process_noise  = gsl_matrix_calloc (2, 2);
    process_noise_linear_acc(Variance_acceleration, control_matrix, process_noise);

    // predict covariance
    gsl_matrix * predicted_covariance= gsl_matrix_calloc (2, 2);
    predict_covariance(state_transition_matrix, covariance_matrix, process_noise, predicted_covariance);

    // formulate observation_matrix
    gsl_matrix * observation_matrix= gsl_matrix_calloc (1, 2);
    observation_matrix_linear_pos_vel(observation_matrix);

    // formulate measurement_noise
    gsl_matrix * measurement_noise= gsl_matrix_calloc (1, 1);
    float variance_measurement[] = {0.11};
    measurement_noise_linear_pos_vel(variance_measurement, measurement_noise);

    //initializing the list of measurements
    float measurement_array[10] = {10.11, 12.16, 12.9, 15.67, 16.2, 17.53, 19.1, 20, 21.12, 22.25};
    
    printf("\n Recorded measurements are:\n");
    for (i = 0; i < 10; i++)
    {
        printf("%.3f \n",measurement_array[i]);
    }
    printf("\n----------------------------------------------------------------\n");
    
    gsl_matrix * kalman_gain = gsl_matrix_calloc (2, 1);

    gsl_vector * updated_state = gsl_vector_calloc (2);
    
    gsl_matrix * updated_covariance = gsl_matrix_calloc (2, 2);
    

    for(i=0; i<10; i++)
    {
        gsl_vector * measurement_vector = gsl_vector_alloc (1);
        gsl_vector_set (measurement_vector, 0, measurement_array[i]);

        // calculate kalman gain
        compute_kalman_gain(predicted_covariance, observation_matrix, measurement_noise, kalman_gain);

        // update state
        update_state(predicted_state, kalman_gain, observation_matrix, measurement_vector, updated_state);
        //gsl_vector_free (measurement_vector);

        // update covariance
        update_covariance(predicted_covariance, kalman_gain, observation_matrix, updated_covariance);

        // predict state
        predict_state(state_transition_matrix, updated_state, control_matrix, control, predicted_state);

        // predict covariance
        predict_covariance(state_transition_matrix, updated_covariance, process_noise, predicted_covariance);

        gsl_vector_free (measurement_vector);
   
    }

    printf("\n Predicted state:\n");
    for (i = 0; i < 2; i++)
    {
      printf ("%g\t", gsl_vector_get (predicted_state, i));
      printf("\n");
    }

    printf("\n Predicted covariance:\n");
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 2; j++)
        {
            printf ("%g\t", gsl_matrix_get (predicted_covariance, i, j));
        }
        printf("\n");
    } 

}



