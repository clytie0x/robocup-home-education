#include "check.h"

int checks_run = 0;
int checks_failed = 0;

int main(void)
{
    test_protocol();
    test_control();
    test_odom();
    test_safety();
    test_app();
    printf("%d checks, %d failed\n", checks_run, checks_failed);
    return checks_failed ? 1 : 0;
}
