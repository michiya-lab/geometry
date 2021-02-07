#include <gtest/gtest.h>

#include <string>

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
namespace
{
    std::vector<std::string> gtest_args;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
TEST(UNIT_TEST_01, PASS_CASE01)
{
    EXPECT_GE(::gtest_args.size(), (std::size_t)1);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    ::gtest_args.resize(argc);
    for (int i = 0; i < argc; ++i)
    {
        ::gtest_args[i] = argv[i];
    } // for i
    return RUN_ALL_TESTS();
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
