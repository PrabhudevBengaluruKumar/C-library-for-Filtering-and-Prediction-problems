#include <check.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include "kalmanLibrary.h"

 #ifdef ck_assert_double_eq_tol
 # define ck_assert_flt_eq(X, Y) ck_assert_double_eq_tol(X, Y, 0.0001)
 #else
 # define ck_assert_flt_eq(X, Y) do { \
   double _dist = fabs((double)(X) - (double)(Y)); \
   ck_assert_msg(_dist < (0.0001), "Assertion '%s' failed: %s == %f,%s == %f", #X" == "#Y, #X, (X), #Y, (Y)); \
 } while (0)
 #endif

START_TEST(test_state_prediction)
{
	
    gsl_vector * state = gsl_vector_alloc(2); 
    gsl_vector_set(state, 0, 10);
    gsl_vector_set(state, 1, 3);

    gsl_vector * control = gsl_vector_alloc(1);
    gsl_vector_set(control, 0, 4);

    gsl_matrix * state_transition = gsl_matrix_alloc(2, 2);
    gsl_matrix_set(state_transition, 0, 0, 1);
    gsl_matrix_set(state_transition, 0, 1, 2);
    gsl_matrix_set(state_transition, 1, 0, 0);
    gsl_matrix_set(state_transition, 1, 1, 1);
 
    gsl_matrix * control_matrix = gsl_matrix_alloc(2, 1);
    gsl_matrix_set(control_matrix, 0, 0, 2);
    gsl_matrix_set(control_matrix, 1, 0, 2);

    gsl_vector * predicted_state = gsl_vector_alloc(2);

    predict_state(state_transition, state, control_matrix, control, predicted_state);
  
    ck_assert_ptr_ne(NULL, predicted_state);
    ck_assert_flt_eq(24, gsl_vector_get(predicted_state, 0));
    ck_assert_flt_eq(11, gsl_vector_get(predicted_state, 1));

    gsl_vector_free (state);
    gsl_matrix_free (state_transition);
    gsl_vector_free (control);
    gsl_matrix_free (control_matrix);
   
}
END_TEST

START_TEST(test_uncertainity_prediction)
{
	
    gsl_matrix * state_transition = gsl_matrix_alloc (2, 2);
    gsl_matrix_set(state_transition, 0, 0, 1);
    gsl_matrix_set(state_transition, 0, 1, 2);
    gsl_matrix_set(state_transition, 1, 0, 0);
    gsl_matrix_set(state_transition, 1, 1, 1);
 
    gsl_matrix * covariance_matrix = gsl_matrix_alloc (2, 2);
    gsl_matrix_set_identity(covariance_matrix);

    gsl_matrix * process_uncertainity = gsl_matrix_alloc (2, 2);
    gsl_matrix_set(process_uncertainity, 0, 0, 3);
    gsl_matrix_set(process_uncertainity, 0, 1, 3);
    gsl_matrix_set(process_uncertainity, 1, 0, 3);
    gsl_matrix_set(process_uncertainity, 1, 1, 3);

    gsl_matrix * predicted_covariance = gsl_matrix_alloc (2, 2);

    predict_covariance(state_transition, covariance_matrix, process_uncertainity, predicted_covariance);
  
    ck_assert_ptr_ne(NULL, predicted_covariance);
    ck_assert_flt_eq(8.0, gsl_matrix_get(predicted_covariance, 0, 0));
    ck_assert_flt_eq(5.0, gsl_matrix_get(predicted_covariance, 0, 1));
    ck_assert_flt_eq(5.0, gsl_matrix_get(predicted_covariance, 1, 0));
    ck_assert_flt_eq(4.0, gsl_matrix_get(predicted_covariance, 1, 1));

    gsl_matrix_free(state_transition);
    gsl_matrix_free(covariance_matrix);
    gsl_matrix_free(process_uncertainity);
    gsl_matrix_free(predicted_covariance);
   
}
END_TEST

START_TEST(test_kalman_gain_computation)
{
    
	gsl_matrix * predicted_covariance = gsl_matrix_alloc (2, 2);
    gsl_matrix_set(predicted_covariance, 0, 0, 8);
    gsl_matrix_set(predicted_covariance, 0, 1, 5);
    gsl_matrix_set(predicted_covariance, 1, 0, 5);
    gsl_matrix_set(predicted_covariance, 1, 1, 4);
 
    gsl_matrix * observation_matrix= gsl_matrix_calloc (1, 2);
    gsl_matrix_set(observation_matrix, 0, 0, 1);
    gsl_matrix_set(observation_matrix, 0, 1, 0);

    gsl_matrix * measurement_uncertainity = gsl_matrix_alloc (1, 1);
    gsl_matrix_set(measurement_uncertainity, 0, 0, 2);

    gsl_matrix * kalman_gain = gsl_matrix_alloc (2, 1);

    compute_kalman_gain(predicted_covariance, observation_matrix, measurement_uncertainity, kalman_gain);
  
    ck_assert_ptr_ne(NULL, kalman_gain);
    ck_assert_flt_eq(0.8, gsl_matrix_get(kalman_gain, 0, 0));   
    ck_assert_flt_eq(0.5, gsl_matrix_get(kalman_gain, 1, 0));   

    gsl_matrix_free(predicted_covariance);
    gsl_matrix_free(observation_matrix);
    gsl_matrix_free(measurement_uncertainity);
    gsl_matrix_free(kalman_gain);
   
}
END_TEST

START_TEST(test_state_updation)
{
	
    gsl_vector * predicted_state = gsl_vector_alloc (2);
    gsl_vector_set(predicted_state, 0, 24);
    gsl_vector_set(predicted_state, 1, 11);
 
    gsl_matrix * observation_matrix= gsl_matrix_calloc (1, 2);
    gsl_matrix_set(observation_matrix, 0, 0, 1);
    gsl_matrix_set(observation_matrix, 0, 1, 0);

    gsl_matrix * kalman_gain = gsl_matrix_alloc (2, 1);
    gsl_matrix_set(kalman_gain, 0, 0, 0.8);
    gsl_matrix_set(kalman_gain, 1, 0, 0.5);

    gsl_vector * measurement_vector = gsl_vector_alloc (1);
    gsl_vector_set(measurement_vector, 0, 26);

    gsl_vector * updated_state = gsl_vector_alloc (2);

    update_state(predicted_state, kalman_gain, observation_matrix, measurement_vector, updated_state);
  
    ck_assert_ptr_ne(NULL, updated_state);
    ck_assert_flt_eq(25.6, gsl_vector_get(updated_state, 0));   
    ck_assert_flt_eq(12.0, gsl_vector_get(updated_state, 1));   

    gsl_vector_free(predicted_state);
    gsl_matrix_free(observation_matrix);
    gsl_matrix_free(kalman_gain);
    gsl_vector_free(measurement_vector);
    gsl_vector_free(updated_state);
    
}
END_TEST

START_TEST(test_uncertainity_updation)
{
	
    gsl_matrix * predicted_covariance = gsl_matrix_alloc(2, 2);
    gsl_matrix_set(predicted_covariance, 0, 0, 8);
    gsl_matrix_set(predicted_covariance, 0, 1, 5);
    gsl_matrix_set(predicted_covariance, 1, 0, 5);
    gsl_matrix_set(predicted_covariance, 1, 1, 4);
 
    gsl_matrix * observation_matrix= gsl_matrix_calloc(1, 2);
    gsl_matrix_set(observation_matrix, 0, 0, 1);
    gsl_matrix_set(observation_matrix, 0, 1, 0);

    gsl_matrix * kalman_gain = gsl_matrix_alloc(2, 1);
    gsl_matrix_set(kalman_gain, 0, 0, 0.8);
    gsl_matrix_set(kalman_gain, 1, 0, 0.5);

    gsl_matrix * Identity_matrix = gsl_matrix_calloc (2, 2);
    gsl_matrix_set_identity(Identity_matrix);

    gsl_matrix * updated_covariance = gsl_matrix_alloc(2, 2);

    update_covariance( predicted_covariance, kalman_gain, observation_matrix, Identity_matrix, updated_covariance);
  
    ck_assert_ptr_ne(NULL, updated_covariance);
    ck_assert_flt_eq(1.6, gsl_matrix_get(updated_covariance, 0, 0));
    ck_assert_flt_eq(1.0, gsl_matrix_get(updated_covariance, 0, 1));
    ck_assert_flt_eq(1.0, gsl_matrix_get(updated_covariance, 1, 0));
    ck_assert_flt_eq(1.5, gsl_matrix_get(updated_covariance, 1, 1));   

    gsl_matrix_free(predicted_covariance);
    gsl_matrix_free(observation_matrix);
    gsl_matrix_free(kalman_gain);
    gsl_matrix_free(updated_covariance);

}
END_TEST

Suite * create_suite_kalman_operations(void)
{
	
    Suite *suite;
    TCase *tcase;

    suite = suite_create("Filters library");

    tcase = tcase_create("Kalman filter");
    tcase_add_test(tcase, test_state_prediction);

    tcase_add_test(tcase, test_uncertainity_prediction);

    tcase_add_test(tcase, test_kalman_gain_computation);

    tcase_add_test(tcase, test_state_updation);

    tcase_add_test(tcase, test_uncertainity_updation);

    suite_add_tcase(suite, tcase);

    return suite;
}

int main(void) {
	
    int number_failed;

    Suite *suite;
    SRunner *suiteRunner;

    suite = create_suite_kalman_operations();
    suiteRunner = srunner_create(suite);

    srunner_run_all(suiteRunner, CK_NORMAL);
    number_failed = srunner_ntests_failed(suiteRunner);
    srunner_free(suiteRunner);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
