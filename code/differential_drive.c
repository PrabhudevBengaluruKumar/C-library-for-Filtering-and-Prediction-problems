#include <stdio.h>
#include <math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_splinalg.h>
#include <gsl/gsl_math.h>
#include "differential_drive.h"

void state_transition_matrix_diff_drive(float delta_t, float wheel_base, gsl_vector * state, gsl_vector * control, gsl_matrix * state_transition)
{
    int i,j;
    int n_x  = state_transition->size1; 
    float array[n_x][n_x], v, a, b, theta, A, B, C, d, w, r;

    v = gsl_vector_get(control, 0);
    a = gsl_vector_get(control, 1);
    theta = gsl_vector_get(state, 2);

    A = ((delta_t * v * tan(a))/w)+theta;

    for(i=0; i<n_x; i++)
    {
        for(j=0; j<n_x; j++)
        {
             if(i == j)
            {
                array[i][j] = 1;
            }
            else if(j == 2){
                if(i == 0){
                    array[i][j] = (wheel_base*(cos(A)-cos(theta)))/tan(a);
                }
                if(i == 1){
                    array[i][j] = (wheel_base*(sin(A)-sin(theta)))/tan(a);
                }
                
            }else   
            {
                array[i][j] = 0;
            }
        }
    }

    for (i = 0; i < n_x; i++)
    {
        for (j = 0; j < n_x; j++)
        {
            gsl_matrix_set (state_transition, i, j, array[i][j]);
        }
    }

}

void control_matrix_diff_drive(float delta_t, float wheel_base, gsl_vector * state, gsl_vector * control, gsl_matrix * control_matrix)
{
    int i,j;
    int n_x  = control_matrix->size1;
    int n_u  = control_matrix->size2; 
    float array[n_x][n_x], v, a, b, theta, A, B, d, w, r;  

    v = gsl_vector_get(control, 0);
    a = gsl_vector_get(control, 1);
    theta = gsl_vector_get(state, 2);

    d = v * delta_t;
    b = (d / wheel_base)* tan(a);
    r = wheel_base / tan(a);

    A = ((delta_t * v * tan(a))/wheel_base)+theta;
    B = pow(tan(a),2) + 1;

    array[0][0] = delta_t * cos(A);
    array[0][1] = delta_t * sin(A);
    array[1][0] = ((delta_t * v * B * cos(A)) / tan(a)) + ((wheel_base * B * sin(theta)) / pow(tan(a),2)) - ((wheel_base * B * sin(A)) / pow(tan(a),2));
    array[1][1] = ((delta_t * v * B * sin(A)) / tan(a)) - ((wheel_base * B * cos(theta)) / pow(tan(a),2)) + ((wheel_base * B * cos(A)) / pow(tan(a),2));
    array[2][0] = (delta_t * tan(a)) / wheel_base;
    array[2][1] = (delta_t * v * B) / wheel_base;

    for (i = 0; i < n_x; i++)
    {
        for (j = 0; j < n_u; j++)
        {
            gsl_matrix_set (control_matrix, i, j, array[i][j]);
        }
    }

}

void process_uncertainity_diff_drive(float variance[], gsl_vector * control, gsl_matrix * control_matrix ,gsl_matrix * process_uncertainity)
{
    int i,j;
    int n_x  = control_matrix->size1;
    int n_u  = control_matrix->size2;
    float array[n_u][n_u], control_input;

    gsl_matrix * noise = gsl_matrix_calloc (n_u, n_u);

    gsl_matrix * C_Ct = gsl_matrix_calloc (n_x, n_u);
    
    for(i=0; i<n_u; i++)
    {
        for(j=0; j<n_u; j++)
        {
             if(i == j)
            {
                control_input = gsl_vector_get(control, i);
                array[i][j] = variance[i]*pow(control_input, 2);
            }
            else
            {
                array[i][j] = 0;
            }
        }
    }

    for (i = 0; i < n_u; i++)
    {
        for (j = 0; j < n_u; j++)
        {
            gsl_matrix_set(noise, i, j, array[i][j]);
        }
    }

    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, control_matrix, noise, 0.0, C_Ct);
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1, C_Ct, control_matrix, 0.0, process_uncertainity);

}

void observation_matrix_diff_drive(float landmark_positions[], gsl_vector * state, gsl_matrix * observation_matrix)
{
    int i,j;
    int n_z = observation_matrix->size1;
    int n_x = observation_matrix->size2;

    float p_x = landmark_positions[0];
    float p_y = landmark_positions[1];

    float x = gsl_vector_get(state, 0);
    float y = gsl_vector_get(state, 1);

    float array[n_z][n_x], A, B, C;

    A = p_x - x;
    B = p_y - y; 
    C = pow(p_x - x, 2) + pow(p_y - y, 2);


    array[0][0] = -A / sqrt(C);
    array[0][1] = -B / sqrt(C);
    array[0][2] = 0;
    array[1][0] = B / C;
    array[1][1] = -A / C;
    array[1][2] = -1;


    for (i = 0; i < n_z; i++)
    {
        for (j = 0; j < n_x; j++)
        {
            gsl_matrix_set (observation_matrix, i, j, array[i][j]);
        }
    }

}

void measurement_uncertainity_diff_drive(float variance[], gsl_matrix * measurement_uncertainity)
{
    int i,j;
    int n_z = measurement_uncertainity->size1;
    
    float array[n_z][n_z];

    for (i=0; i<n_z; i++)
    {
        for (j=0; j<n_z; j++)
        {
            if (i == j)
            {
                array[i][j] = variance[i];
            }else
            {
                array[i][j] = 0;
            }
        }
    }

    for (i = 0; i < n_z; i++)
    {
        for (j = 0; j < n_z; j++)
        {
            gsl_matrix_set(measurement_uncertainity, i, j, array[i][j]);
        }
    }
}

