/* A tiny test helper: count checks, print each failure with its line, never stop early. */
#ifndef CHECK_H
#define CHECK_H

#include <math.h>
#include <stdio.h>
#include <string.h>

extern int checks_run;
extern int checks_failed;

#define CHECK(cond)                                                       \
    do {                                                                  \
        checks_run++;                                                     \
        if (!(cond)) {                                                    \
            checks_failed++;                                              \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                             \
    do {                                                                  \
        checks_run++;                                                     \
        float got_ = (float)(a), want_ = (float)(b);                      \
        if (!(fabsf(got_ - want_) <= (float)(tol))) {                     \
            checks_failed++;                                              \
            printf("FAIL %s:%d: %s = %.6f, want %.6f +- %g\n", __FILE__,  \
                   __LINE__, #a, (double)got_, (double)want_, (double)(tol)); \
        }                                                                 \
    } while (0)

#define CHECK_STR(got, want)                                              \
    do {                                                                  \
        checks_run++;                                                     \
        if (strcmp((got), (want)) != 0) {                                 \
            checks_failed++;                                              \
            printf("FAIL %s:%d: got \"%s\", want \"%s\"\n", __FILE__,     \
                   __LINE__, (got), (want));                              \
        }                                                                 \
    } while (0)

void test_protocol(void);
void test_control(void);
void test_odom(void);
void test_safety(void);
void test_params(void);
void test_boot(void);
void test_app(void);

#endif
