#include <stdio.h>
#include <math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_splinalg.h>
#include <gsl/gsl_math.h>
#include "linear.h"
// function to formulate the state transition matrix to estimate position and velocity for a linear system 
void state_transition_linear_pos_vel(float delta_t, gsl_matrix * state_transition_matrix)
{
    int i,j;
    int n  = state_transition_matrix->size2;
    int dim = n/2;
    float array[n][n];
    for(i=0; i<n; i++)
    {
        for(j=0; j<n; j++)
        {
            if(i == j)
            {
                array[i][j] = 1;
            }
            else if( dim+i == j)
            {
                array[i][j] = delta_t;
            }else
            {
                array[i][j] = 0;
            }
        }
    }  
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            gsl_matrix_set (state_transition_matrix, i, j, array[i][j]);
        }
    }
}
// function to formulate the control matrix for a linear system with acceleration as input
void control_matrix_linear_acc(float delta_t, gsl_matrix * control_matrix)
{
    int i,j;
    int m = control_matrix->size1;
    int n = control_matrix->size2;
    
    float array[m][n];
    for(i=0; i<m; i++)
    {
        for(j=0; j<n; j++)
        {
            if(i == j)
            {
                array[i][j] = 0.5 * pow(delta_t,2);
            }
            else if( j == i-n)
            {
                array[i][j] = delta_t;
            }else
            {
                array[i][j] = 0;
            }
        }
    }
    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            gsl_matrix_set (control_matrix, i, j, array[i][j]);
        }
    }
}
// function to formulate the process uncertainity a linear system with acceleration as input
void process_uncertainity_linear_acc(float variance, gsl_matrix * control_matrix, gsl_matrix * process_uncertainity)
{
    int i,j;
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, variance, control_matrix, control_matrix, 0.0, process_uncertainity);
 
}
// function to formulate the observation matrix to reshape to states to match measurement for a linear system 
void observation_matrix_linear_pos_vel(gsl_matrix * observation_matrix)
{
    int i,j;
    int m = observation_matrix->size1;
    int n = observation_matrix->size2;
    float array[m][n];
    for(i=0; i<m; i++)
    {
        for(j=0; j<n; j++)
        {
            if(i == j)
            {
                array[i][j] = 1;
            }else
            {
                array[i][j] = 0;
            }
        }
    }
    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            gsl_matrix_set (observation_matrix, i, j, array[i][j]);
        }
    }
}
// function to formulate the meaurement uncertainity matrix for the measured position and velocity in a linear system 
void measurement_uncertainity_linear_pos_vel(const float Variance[], gsl_matrix * measurement_uncertainity)
{
    int i,j;
    int m = measurement_uncertainity->size1;
    
    float array[m][m];
    for (i=0; i<m; i++)
    {
        for (j=0; j<m; j++)
        {
            if (i == j)
            {
                array[i][j] = Variance[i];
            }else
            {
                array[i][j] = 0;
            }
        }
    }
    for (i = 0; i < m; i++)
    {
        for (j = 0; j < m; j++)
        {
            gsl_matrix_set (measurement_uncertainity, i, j, array[i][j]);
        }
    }
}
