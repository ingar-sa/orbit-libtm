/*
    Minimal unittesting framework, inspired by MinUnit
   (https://jera.com/techinfo/jtns/jtn002).
*/

#ifndef ORBU_USE_STDLIB
#define ORBU_USE_STDLIB 1
#endif

#ifndef ORBU_USE_ZEPHYR
#define ORBU_USE_ZEPHYR 0
#endif

#if ORBU_USE_STDLIB
#include <stddef.h>
#include <stdio.h>

#define ORBU_PRINTF(...) fprintf(stdout, __VA_ARGS__)
#define ORBU_ERRF(...) fprintf(stderr, __VA_ARGS__)
#elif ORBU_USE_ZEPHYR
#include <logging/log.h>
#include <zephyr.h>

#define ORBU_PRINTF(...) LOG_INF(__VA_ARGS__)
#define ORBU_ERRF(...) LOG_ERR(__VA_ARGS__)
#else
#error "missing substitutions for stdlib functions"
#endif

typedef int (*orbu_test_case)(void);

#define __orbu_assertion_fail(check, ...)                                      \
    do {                                                                       \
        ORBU_ERRF("\033[0;31m<%s (%i): %s failed>: ", __func__, __LINE__,      \
                  check);                                                      \
        ORBU_ERRF(__VA_ARGS__);                                                \
        ORBU_ERRF("\n\033[0m");                                                \
        return 1;                                                              \
    } while (0)

#define __orbu_assert_helper(check, cond, msg)                                 \
    do {                                                                       \
        if (cond) {                                                            \
            break;                                                             \
        }                                                                      \
        __orbu_assertion_fail(check, msg);                                     \
    } while (0)

#define orbu_assert(cond, msg) __orbu_assert_helper("orbu_assert", cond, msg)

#define orbu_run_test(test, result)                                            \
    do {                                                                       \
        result = test() || result;                                             \
    } while (0)

#define orbu_run_all_tests(tests)                                              \
    do {                                                                       \
        int num_tests = 0, num_failed = 0;                                     \
        for (size_t i = 0; i < sizeof(tests) / sizeof(*tests); i++) {          \
            orbu_test_case test = tests[i];                                    \
            num_failed += test();                                              \
            num_tests++;                                                       \
        }                                                                      \
        ORBU_PRINTF("%s<%s (%i)> SUMMARY: Ran %i tests, where %i of them "     \
                    "failed\n\033[0m",                                         \
                    num_failed != 0 ? "\033[0;33m" : "\033[0;32m", __FILE__,   \
                    __LINE__, num_tests, num_failed);                          \
        return num_failed != 0;                                                \
    } while (0)
