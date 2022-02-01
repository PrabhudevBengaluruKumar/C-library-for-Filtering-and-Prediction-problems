#include<check.h>
#include<stdlib.h>
#include<float.h>
#include<math.h>
#include "linear.h"
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
    gsl_matrix * state_transition_matrix = gsl_matrix_calloc (2, 2);

    state_transition_linear_pos_vel(0.5, state_transition_matrix);
  
    ck_assert_ptr_ne(NULL, state_transition_matrix);
    ck_assert_flt_eq(1.0, gsl_matrix_get(state_transition_matrix, 0, 0));
    ck_assert_flt_eq(0.5, gsl_matrix_get(state_transition_matrix, 0, 1));
    ck_assert_flt_eq(0.0, gsl_matrix_get(state_transition_matrix, 1, 0));
    ck_assert_flt_eq(1.0, gsl_matrix_get(state_transition_matrix, 1, 1));

    gsl_matrix_free (state_transition_matrix);
}
END_TEST

START_TEST(test_control_matrix)
{
    gsl_matrix * control_matrix = gsl_matrix_calloc (2, 1);

    control_matrix_linear_acc(0.5, control_matrix);
  
    ck_assert_ptr_ne(NULL, control_matrix);
    ck_assert_flt_eq(0.125, gsl_matrix_get(control_matrix, 0, 0));
    ck_assert_flt_eq(0.5, gsl_matrix_get(control_matrix, 1, 0));

    gsl_matrix_free (control_matrix);
}
END_TEST

START_TEST(test_process_uncertainity)
{
    gsl_matrix * process_uncertainity  = gsl_matrix_calloc (2, 2);

    gsl_matrix * control_matrix = gsl_matrix_alloc(2, 1);
    gsl_matrix_set(control_matrix, 0, 0, 0.125);
    gsl_matrix_set(control_matrix, 1, 0, 0.5);

    process_uncertainity_linear_acc(0.75, control_matrix, process_uncertainity);
  
    ck_assert_ptr_ne(NULL, process_uncertainity);
    ck_assert_flt_eq(0.011719, gsl_matrix_get(process_uncertainity, 0, 0));
    ck_assert_flt_eq(0.046875, gsl_matrix_get(process_uncertainity, 1, 0));
    ck_assert_flt_eq(0.046875, gsl_matrix_get(process_uncertainity, 0, 1));
    ck_assert_flt_eq(0.187500, gsl_matrix_get(process_uncertainity, 1, 1));

    gsl_matrix_free (control_matrix);
    gsl_matrix_free (process_uncertainity);
}
END_TEST

START_TEST(test_observation_matrix)
{
    gsl_matrix * observation_matrix= gsl_matrix_calloc (1, 2);

    observation_matrix_linear_pos_vel(observation_matrix);
  
    ck_assert_ptr_ne(NULL, observation_matrix);
    ck_assert_flt_eq(1, gsl_matrix_get(observation_matrix, 0, 0));
    ck_assert_flt_eq(0, gsl_matrix_get(observation_matrix, 0, 1));

    gsl_matrix_free (observation_matrix);
}
END_TEST

START_TEST(test_measurement_uncertainity)
{
    gsl_matrix * measurement_uncertainity= gsl_matrix_calloc (1, 1);

    float variance_measurement[] = {2};

    measurement_uncertainity_linear_pos_vel(variance_measurement, measurement_uncertainity);
  
    ck_assert_ptr_ne(NULL, measurement_uncertainity);
    ck_assert_flt_eq(2.0, gsl_matrix_get(measurement_uncertainity, 0, 0));

    gsl_matrix_free (measurement_uncertainity);
}
END_TEST


Suite * create_suite_linear_operations(void)
{
    Suite *suite;
    TCase *tcase;

    suite = suite_create("linear composable operations");

    tcase = tcase_create("Operations");

    tcase_add_test(tcase, test_state_transition);
    
    tcase_add_test(tcase, test_control_matrix);

    tcase_add_test(tcase, test_process_uncertainity);

    tcase_add_test(tcase, test_observation_matrix);

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
