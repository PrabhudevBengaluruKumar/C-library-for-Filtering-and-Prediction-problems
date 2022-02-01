//Calculating the position and velocity of a body in 2-Dimension

#include<stdio.h>
#include "differential_drive.h"
#include "kalmanLibrary.h"

void main()
{
    int i, j, k;

    printf("\n------------------------- SAMPLE CASE 5 ------------------------");
    printf("\n Estimating the position and orientation of a differential drive");
    printf("\n----------------------------------------------------------------\n");

     
    float *state_array=malloc(4 * sizeof(float));

    //initial position x , y and initial orientation 
    state_array[0] = 10;
    state_array[1] = 5;
    state_array[2] = 0.75;
    
    // allocating memory and initializing state variables
    gsl_vector * state = gsl_vector_alloc (3);
    for (i = 0; i < 3; i++)
    {
      gsl_vector_set (state, i, state_array[i]);
    } 
    
    printf("\n The initial position and orientation at which the differential drive starts is:\n");
    for (i = 0; i < 3; i++)
    {
      printf ("%g\n", gsl_vector_get (state, i));
      printf("\n");
    }
    free(state_array);

    printf("\n----------------------------------------------------------------\n");

    float *control_array=malloc(2 * sizeof(float));
    
    //initial velocity and steering angle
    control_array[0] = 2.5;
    control_array[1] = 0.1;

    // allocating memory for control variables
    gsl_vector * control = gsl_vector_alloc (2);
    for (i = 0; i < 2; i++)
    {
        gsl_vector_set (control, i, control_array[i]);
    }
    printf("\n The velocity and steering angle at which the differential is moving is: \n");
    for (i = 0; i < 2; i++)
    {
      printf ("%g\n", gsl_vector_get (control, i));
    }
    free(control_array);

    printf("\n----------------------------------------------------------------\n"); 

    float delta_t = 0.5;
    printf("\n The time interval at which the measurements are made is: %f", delta_t);

    printf("\n----------------------------------------------------------------\n");

    float wheel_base = 4;

    // formulating state transition matrix
    gsl_matrix * state_transition_matrix = gsl_matrix_calloc (3, 3);
    state_transition_matrix_diff_drive(delta_t, wheel_base, state, control, state_transition_matrix);
    printf("\n1\n");

   // formulating control matrix
    gsl_matrix * control_matrix = gsl_matrix_calloc (3, 2);
    control_matrix_diff_drive(delta_t, wheel_base, state, control, control_matrix);
    printf("\n2\n");

   // allocating memory for uncertainity matrix
    gsl_matrix * covariance_matrix = gsl_matrix_calloc (3, 3);
    gsl_matrix_set_identity(covariance_matrix);
    printf("\n3\n");

    // predict state
    gsl_vector * predicted_state = gsl_vector_calloc (3);
    predict_state(state_transition_matrix, state, control_matrix, control, predicted_state);
    printf("\n4\n");

    // formulating process uncertainity matrix
    float variance_process[2] = {0.25, 0.1};
    gsl_matrix * process_uncertainity  = gsl_matrix_calloc (3, 3);
    process_uncertainity_diff_drive(variance_process, control, control_matrix , process_uncertainity);
    printf("\n5\n");

    // predict covariance
    gsl_matrix * predicted_covariance= gsl_matrix_calloc (3, 3);
    predict_covariance(state_transition_matrix, covariance_matrix, process_uncertainity, predicted_covariance);
    printf("\n6\n");

    //initializing the list of measurements?xe
    float landmark_positions[10][2] = {{10.11, 5.99}, {12.16,7.12}, {12.9,8}, {15.67,9.5}, {16.2,11.33}, {17.53,12.5}, {19.1,12.9}, {20,14.02}, {21.12, 14.89}, {22.25,15.76}};
    float landmark[2];
    for(j=0; j<2; j++){
        landmark[j] = landmark_positions[0][j];
    }

    // formulate observation_matrix
    gsl_matrix * observation_matrix= gsl_matrix_calloc (2, 3);
    observation_matrix_diff_drive(landmark, state, observation_matrix);

    // formulate measurement_uncertainity
    gsl_matrix * measurement_uncertainity = gsl_matrix_calloc (2, 2);
    float variance_measurement[] = {0.11, 0.23};
    measurement_uncertainity_diff_drive(variance_measurement, measurement_uncertainity);

    gsl_matrix * kalman_gain = gsl_matrix_calloc (3, 2);
    gsl_vector * updated_state = gsl_vector_calloc (3);
    gsl_matrix * updated_covariance = gsl_matrix_calloc (3, 3);
    gsl_matrix * Identity_matrix = gsl_matrix_calloc (3, 3);
    gsl_matrix_set_identity(Identity_matrix);

    printf("\n Recorded measurements are:\n");
    for (i = 0; i < 10; i++)
    {
        for(j=0; j<2; j++){
            printf("%.3f \t",landmark_positions[i][j]);
        }
        printf("\n");
    }
    printf("\n----------------------------------------------------------------\n");

    for(i=0; i<10; i++)
    {
        gsl_vector * measurement_vector = gsl_vector_alloc (2);
        for(j=0; j<2; j++){
            gsl_vector_set (measurement_vector, j, landmark_positions[i][j]);
        }
        
	//compute kalman gain
        compute_kalman_gain( predicted_covariance, observation_matrix, measurement_uncertainity, kalman_gain);
        
        // update state
        update_state(predicted_state, kalman_gain, observation_matrix, measurement_vector, updated_state);

        // update covariance
        update_covariance(predicted_covariance, kalman_gain, observation_matrix, Identity_matrix, updated_covariance); 
        
        // predict state
        predict_state(state_transition_matrix, updated_state, control_matrix, control, predicted_state);

        // predict covariance
        predict_covariance(state_transition_matrix, updated_covariance, process_uncertainity, predicted_covariance);

        gsl_vector_free(measurement_vector);
   
    }

    printf("\n Predicted state:\n");
    for (i = 0; i < 3; i++)
    {
      printf ("%g \t", gsl_vector_get (predicted_state, i));
      printf("\n");
    }

    printf("\n Predicted covariance:\n");
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            printf ("%g \t", gsl_matrix_get (predicted_covariance, i, j));
        }
        printf("\n");
    } 

}



