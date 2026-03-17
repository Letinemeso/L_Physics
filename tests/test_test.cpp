#include <Testing/Testing.h>

INIT_TEST(Ass_Test)
{
    int a = 10;
    EXPECT_LESS(a, 9);
}
