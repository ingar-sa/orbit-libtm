
#include "../inc/orbunit.h"

int test_init(void);
int test_switch_file(void);
int test_buffer_write(void);
int test_file_write(void);

static orbu_test_case tests[] = {test_init, test_switch_file, test_buffer_write, test_file_write};

int main(void) { orbu_run_all_tests(tests);}