#ifndef TESTS_HOST_SUPPORT_H
#define TESTS_HOST_SUPPORT_H

void test_check(int condition, const char *expression, const char *file, unsigned line);
#define CHECK(condition) test_check(!!(condition), #condition, __FILE__, __LINE__)

#endif
