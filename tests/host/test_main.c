#include "test_support.h"

#include <stdio.h>

static unsigned passed, failed;

void test_boot_run(void);
void test_command_run(void);
void test_release_run(void);
void test_fw_update_run(void);

void test_check(int condition, const char *expression, const char *file, unsigned line)
{
    if (condition) {
        passed++;
    } else {
        failed++;
        printf("FAIL %s:%u: %s\n", file, line, expression);
    }
}

int main(void)
{
    test_boot_run();
    test_command_run();
    test_release_run();
    test_fw_update_run();
    printf("host tests: %u passed, %u failed\n", passed, failed);
    return failed != 0u;
}
