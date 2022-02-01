#include<check.h>
#include<stdlib.h>
#include<float.h>
#include<math.h>
#include "differential_drive.h"
#include "kalmanLibrary.h"

 #ifdef ck_assert_double_eq_tol
 # define ck_assert_flt_eq(X, Y) ck_assert_double_eq_tol(X, Y, 0.0001)
 #else
 # define ck_assert_flt_eq(X, Y) do { \
   double _dist = fabs((double)(X) - (double)(Y)); \
   ck_assert_msg(_dist < (0.0001), "Assertion '%s' failed: %s == %f,%s == %f", #X" == "#Y, #X, (X), #Y, (Y)); \
 } while (0)
 #endif

START_TEST(test_state_transition)
{
    gsl_matrix * state_transition_matrix = gsl_matrix_calloc (3, 3);

    gsl_vector * state = gsl_vector_alloc(3); 
    gsl_vector_set(state, 0, 10);
    gsl_vector_set(state, 1, 3);
    gsl_vector_set(state, 2, 4);

    gsl_vector * control = gsl_vector_alloc(2); 
    gsl_vector_set(control, 0, 5);
    gsl_vector_set(control, 1, 6);

    state_transition_matrix_diff_drive(0.5, 2.0, state, control, state_transition_matrix);
  
    ck_assert_ptr_ne(NULL, state_transition_matrix);
    ck_assert_flt_eq(1.0, gsl_matrix_get(state_transition_matrix, 0, 0));
    ck_assert_flt_eq(0.0, gsl_matrix_get(state_transition_matrix, 0, 1));
    ck_assert_flt_eq(0.0, gsl_matrix_get(state_transition_matrix, 1, 0));
    ck_assert_flt_eq(1.0, gsl_matrix_get(state_transition_matrix, 1, 1));
    ck_assert_flt_eq(0.0, gsl_matrix_get(state_transition_matrix, 2, 0));
    ck_assert_flt_eq(0.0, gsl_matrix_get(state_transition_matrix, 2, 1));
    ck_assert_flt_eq(1.0, gsl_matrix_get(state_transition_matrix, 2, 2));

    gsl_vector_free(control);
    gsl_vector_free(state);
    gsl_matrix_free (state_transition_matrix);

}
END_TEST

START_TEST(test_control_matrix)
{
    gsl_matrix * control_matrix = gsl_matrix_calloc (3, 2);

    gsl_vector * state = gsl_vector_alloc(3); 
    gsl_vector_set(state, 0, 10);
    gsl_vector_set(state, 1, 3);
    gsl_vector_set(state, 2, 4);

    gsl_vector * control = gsl_vector_alloc(2); 
    gsl_vector_set(control, 0, 5);
    gsl_vector_set(control, 1, 6);

    control_matrix_diff_drive(0.5, 2.0, state, control, control_matrix);
  
    ck_assert_ptr_ne(NULL, control_matrix);
    ck_assert_flt_eq(-0.440068, gsl_matrix_get(control_matrix, 0, 0));
    ck_assert_flt_eq(-0.237362, gsl_matrix_get(control_matrix, 0, 1));
    ck_assert_flt_eq(0.975411, gsl_matrix_get(control_matrix, 1, 0));
    ck_assert_flt_eq(-1.378384, gsl_matrix_get(control_matrix, 1, 1));
    ck_assert_flt_eq(-0.072752, gsl_matrix_get(control_matrix, 2, 0));
    ck_assert_flt_eq(1.355856, gsl_matrix_get(control_matrix, 2, 1));

    gsl_vector_free(control);
    gsl_vector_free(state);
    gsl_matrix_free(control_matrix);

}
END_TEST

START_TEST(test_process_uncertainity)
{
    gsl_matrix * process_uncertainity  = gsl_matrix_calloc (3, 3);

    float variance[2] = {2, 4};

    gsl_vector * control = gsl_vector_alloc(2); 
    gsl_vector_set(control, 0, 5);
    gsl_vector_set(control, 1, 6);

    gsl_matrix * control_matrix = gsl_matrix_alloc(3, 2);
    gsl_matrix_set(control_matrix, 0, 0, 0.125);
    gsl_matrix_set(control_matrix, 0, 1, 0.5);
    gsl_matrix_set(control_matrix, 1, 0, 0.125);
    gsl_matrix_set(control_matrix, 1, 1, 0.5);
    gsl_matrix_set(control_matrix, 2, 0, 0.125);
    gsl_matrix_set(control_matrix, 2, 1, 0.5);

    process_uncertainity_diff_drive(variance, control, control_matrix , process_uncertainity);
  
    ck_assert_ptr_ne(NULL, process_uncertainity);
    ck_assert_flt_eq(36.781250, gsl_matrix_get(process_uncertainity, 0, 0));
    ck_assert_flt_eq(36.781250, gsl_matrix_get(process_uncertainity, 0, 1));
    ck_assert_flt_eq(36.781250, gsl_matrix_get(process_uncertainity, 0, 2));
    ck_assert_flt_eq(36.781250, gsl_matrix_get(process_uncertainity, 1, 0));
    ck_assert_flt_eq(36.781250, gsl_matrix_get(process_uncertainity, 1, 1));
    ck_assert_flt_eq(36.781250, gsl_matrix_get(process_uncertainity, 1, 2));
    ck_assert_flt_eq(36.781250, gsl_matrix_get(process_uncertainity, 2, 0));
    ck_assert_flt_eq(36.781250, gsl_matrix_get(process_uncertainity, 2, 1));
    ck_assert_flt_eq(36.781250, gsl_matrix_get(process_uncertainity, 2, 2));

    gsl_vector_free(control);
    gsl_matrix_free(control_matrix);
    gsl_matrix_free(process_uncertainity);

}
END_TEST

START_TEST(test_observation_matrix)
{
    gsl_vector * state = gsl_vector_calloc(3); 
    gsl_vector_set(state, 0, 10);
    gsl_vector_set(state, 1, 3);
    gsl_vector_set(state, 2, 4);

    gsl_matrix * observation_matrix= gsl_matrix_calloc(2, 3);
 
    float landmark_positions[2] = {10.11, 5.99};

    observation_matrix_diff_drive(landmark_positions, state, observation_matrix);
  
    ck_assert_ptr_ne(NULL, observation_matrix);
    ck_assert_flt_eq(1, gsl_matrix_get(observation_matrix, 0, 0));
    ck_assert_flt_eq(0, gsl_matrix_get(observation_matrix, 0, 1));
    ck_assert_flt_eq(1, gsl_matrix_get(observation_matrix, 0, 2));
    ck_assert_flt_eq(0, gsl_matrix_get(observation_matrix, 1, 0));
    ck_assert_flt_eq(1, gsl_matrix_get(observation_matrix, 1, 1));
    ck_assert_flt_eq(0, gsl_matrix_get(observation_matrix, 1, 2));

    //gsl_matrix_free (observation_matrix);

}
END_TEST

START_TEST(test_measurement_uncertainity)
{
    gsl_matrix * measurement_uncertainity= gsl_matrix_calloc (2, 2);

    float variance[2] = {2, 4};

    measurement_uncertainity_diff_drive(variance, measurement_uncertainity);
  
    ck_assert_ptr_ne(NULL, measurement_uncertainity);
    ck_assert_flt_eq(2.0, gsl_matrix_get(measurement_uncertainity, 0, 0));
    ck_assert_flt_eq(0.0, gsl_matrix_get(measurement_uncertainity, 0, 1));
    ck_assert_flt_eq(0.0, gsl_matrix_get(measurement_uncertainity, 1, 0));
    ck_assert_flt_eq(4.0, gsl_matrix_get(measurement_uncertainity, 1, 1));

    gsl_matrix_free (measurement_uncertainity);

}
END_TEST


Suite * create_suite_linear_operations(void)
{
    Suite *suite;
    TCase *tcase;

    suite = suite_create("Differential drive: kalman composable operations");

    tcase = tcase_create("Operations");

    tcase_add_test(tcase, test_state_transition);
    
    tcase_add_test(tcase, test_control_matrix);

    tcase_add_test(tcase, test_process_uncertainity);

    //tcase_add_test(tcase, test_observation_matrix);

    tcase_add_test(tcase, test_measurement_uncertainity);
    suite_add_tcase(suite, tcase);

    return suite;
}

int main(void) {
    int number_failed;

    Suite *suite;
    SRunner *suiteRunner;

    suite = create_suite_linear_operations();
    suiteRunner = srunner_create(suite);

    srunner_run_all(suiteRunner, CK_NORMAL);
    number_failed = srunner_ntests_failed(suiteRunner);
    srunner_free(suiteRunner);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
