#include <string>

#include <gtest/gtest.h>

#include <geometry_tools.hpp>

namespace
{
    std::vector<std::string> gtest_args;
}

TEST(UNIT_TEST_01, PASS_CASE01)
{
    EXPECT_GE(::gtest_args.size(), (std::size_t)1);
}

TEST(UnitTest_LengthPontAndLine3D, On2D_1)
{
    // https://www.varsitytutors.com/precalculus-help/find-the-distance-between-a-point-and-a-line
    auto query = Eigen::Vector3d(-2.0e0, -4.0e0, 0.0e0);
    auto p1 = Eigen::Vector3d(0.0e0, 2.0e0, 0.0e0);
    auto p2 = Eigen::Vector3d(3.0e0, 1.0e0, 0.0e0);
    auto dist = geometry::ComputeLengthPoint3dAndLine3d(query, p1, p2);
    EXPECT_DOUBLE_EQ(dist, 2.0e0 * std::sqrt(10.0e0));
}
TEST(UnitTest_LengthPontAndLine3D, On2D_2)
{
    // https://www.varsitytutors.com/precalculus-help/find-the-distance-between-a-point-and-a-line
    auto query = Eigen::Vector3d(0.0e0, 0.0e0, 0.0e0);
    auto p1 = Eigen::Vector3d(0.0e0, -4.0e0, 0.0e0);
    auto p2 = Eigen::Vector3d(1.0e0, -1.0e0, 0.0e0);
    auto dist = geometry::ComputeLengthPoint3dAndLine3d(query, p1, p2);
    EXPECT_DOUBLE_EQ(dist, 2.0e0 * std::sqrt(10.0e0) / 5.0e0);
}
TEST(UnitTest_LengthPontAndLine3D, On3D_1)
{
    // https://onlinemschool.com/math/library/analytic_geometry/p_line/
    auto query = Eigen::Vector3d(0.0e0, 2.0e0, 3.0e0);
    auto p1 = Eigen::Vector3d(5.0e0, 2.0e0, 1.0e0);
    auto p2 = Eigen::Vector3d(1.0e0, -0.0e0, -3.0e0);
    auto dist = geometry::ComputeLengthPoint3dAndLine3d(query, p1, p2);
    EXPECT_DOUBLE_EQ(dist, 5.0e0);
}
TEST(UnitTest_LengthPontAndLine3D, On3D_2)
{
    // https://www.geeksforgeeks.org/shortest-distance-between-a-line-and-a-point-in-a-3-d-plane/
    auto query = Eigen::Vector3d(0.0e0, 2.0e0, 0.0e0);
    auto p1 = Eigen::Vector3d(4.0e0, 2.0e0, 1.0e0);
    auto p2 = Eigen::Vector3d(3.0e0, 2.0e0, 1.0e0);
    auto dist = geometry::ComputeLengthPoint3dAndLine3d(query, p1, p2);
    EXPECT_DOUBLE_EQ(dist, 1.0e0);
}
TEST(UnitTest_ShortestDistanceTwoLine3d, Skew2D)
{
    auto p1 = Eigen::Vector3d(0.0e0, 0.0e0, 0.0e0);
    auto p2 = Eigen::Vector3d(1.0e0, -2.0e0, 0.0e0);
    auto p3 = Eigen::Vector3d(1.0e0, -1.0e0, 0.0e0);
    auto p4 = Eigen::Vector3d(3.0e0, 1.0e0, 0.0e0);
    auto shortest = geometry::ComputeShortestLine3dBetweenLine3dAndLine3d(p1, p2, p3, p4);
    EXPECT_LT((shortest.second - shortest.first).norm(), 1.0e-10);
}
TEST(UnitTest_ShortestDistanceTwoLine3d, Parallel2D)
{
    // https://www.varsitytutors.com/precalculus-help/find-the-distance-between-a-point-and-a-line
    auto p1 = Eigen::Vector3d(0.0e0, 2.0e0, 0.0e0);
    auto p2 = Eigen::Vector3d(3.0e0, 1.0e0, 0.0e0);
    auto p3 = Eigen::Vector3d(-2.0e0, -4.0e0, 0.0e0);
    auto p4 = Eigen::Vector3d(1.0e0, -5.0e0, 0.0e0);
    EXPECT_TRUE(geometry::IsParallel(p1, p2, p3, p4));
    auto shortest = geometry::ComputeShortestLine3dBetweenLine3dAndLine3d(p1, p2, p3, p4);
    EXPECT_DOUBLE_EQ((shortest.second - shortest.first).norm(), 2.0e0 * std::sqrt(10.0e0));
}
TEST(UnitTest_IntersectionTwoLine3d, Paralell_1)
{
    // https://www.mathepower.com/en/lineintersection.php
    EXPECT_GE(::gtest_args.size(), (std::size_t)1);
    auto p1 = Eigen::Vector3d(2.0e0, 3.0e0, 4.0e0);
    auto p2 = Eigen::Vector3d(8.0e0, -1.0e0, 5.0e0);
    auto p3 = Eigen::Vector3d(-3.0e0, -1.0e0, 8.0e0);
    auto p4 = Eigen::Vector3d(10.0e0, -8.0e0, 7.0e0);
    Eigen::Vector3d intersection;
    EXPECT_FALSE(geometry::HasIntersectionOfLine3dSegmentAndLine3dSegment(p1, p2, p3, p4, intersection));
}

TEST(UnitTest_IntersectionTwoLine3d, Skewl_1)
{
    // https://math.stackexchange.com/questions/270767/find-intersection-of-two-3d-lines/271366
    auto p1 = Eigen::Vector3d(5.0e0, 5.0e0, 4.0e0);
    auto p2 = Eigen::Vector3d(10.0e0, 10.0e0, 6.0e0);
    auto p3 = Eigen::Vector3d(5.0e0, 5.0e0, 5.0e0);
    auto p4 = Eigen::Vector3d(10.0e0, 10.0e0, 3.0e0);
    Eigen::Vector3d intersection;
    EXPECT_TRUE(geometry::HasIntersectionOfLine3dSegmentAndLine3dSegment(p1, p2, p3, p4, intersection));
    auto ans = Eigen::Vector3d(6.25e0, 6.25e0, 4.5e0);
    EXPECT_TRUE((ans - intersection).norm() < 1.0e-8);
}

TEST(UnitTest_IntersectionTwoLine3d, ParallelRandom)
{
    // all points must be paralell
    bool success = true;
    for (std::size_t i = 0; i < 1000; i++)
    {
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<double> rand(-1.0e-0, 1.0e-0);
        Eigen::Vector3d p1;
        p1.x() = rand(mt);
        p1.y() = rand(mt);
        p1.z() = rand(mt);
        Eigen::Vector3d p2;
        p2.x() = rand(mt);
        p2.y() = rand(mt);
        p2.z() = rand(mt);
        Eigen::Vector3d p3;
        p3.x() = rand(mt);
        p3.y() = rand(mt);
        p3.z() = rand(mt);
        Eigen::Vector3d p4;
        p4 = p3 + (p2 - p1) * rand(mt);
        if (geometry::IsParallel(p1, p2, p3, p4))
        {
            auto shortest = geometry::ComputeShortestLineBetweenParallelLine3dAndLine3d(p1, p2, p3, p4);
            constexpr double eps = 1.0e-8;
            auto tol = eps * (shortest.second - shortest.first).norm();
            auto dist1 = (shortest.second - shortest.first).norm();
            auto dist2 = geometry::ComputeLengthPoint3dAndLine3d(p1, p3, p4);
            if (std::abs(dist1 - dist2) > tol)
            {
                success = false;
                break;
            }
        }
    }
    EXPECT_TRUE(success);
}
TEST(UnitTest_IntersectionTwoLine3d, ParallelRandom_SmallerOrder)
{
    // all points must be paralell
    bool success = true;
    for (std::size_t i = 0; i < 1000; i++)
    {
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<double> rand(-1.0e-20, 1.0e-20);
        Eigen::Vector3d p1;
        p1.x() = rand(mt);
        p1.y() = rand(mt);
        p1.z() = rand(mt);
        Eigen::Vector3d p2;
        p2.x() = rand(mt);
        p2.y() = rand(mt);
        p2.z() = rand(mt);
        Eigen::Vector3d p3;
        p3.x() = rand(mt);
        p3.y() = rand(mt);
        p3.z() = rand(mt);
        Eigen::Vector3d p4;
        p4 = p3 + (p2 - p1) * rand(mt);
        if (geometry::IsParallel(p1, p2, p3, p4))
        {
            auto shortest = geometry::ComputeShortestLineBetweenParallelLine3dAndLine3d(p1, p2, p3, p4);
            constexpr double eps = 1.0e-8;
            auto tol = eps * (shortest.second - shortest.first).norm();
            auto dist1 = (shortest.second - shortest.first).norm();
            auto dist2 = geometry::ComputeLengthPoint3dAndLine3d(p1, p3, p4);
            if (std::abs(dist1 - dist2) > tol)
            {
                success = false;
                break;
            }
        }
    }
    EXPECT_TRUE(success);
}
TEST(UnitTest_IntersectionTwoLine3d, ParalellRandom_LargerOrder)
{
    // all points must be paralell
    bool success = true;
    for (std::size_t i = 0; i < 1000; i++)
    {
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<double> rand(-1.0e+20, 1.0e+20);
        Eigen::Vector3d p1;
        p1.x() = rand(mt);
        p1.y() = rand(mt);
        p1.z() = rand(mt);
        Eigen::Vector3d p2;
        p2.x() = rand(mt);
        p2.y() = rand(mt);
        p2.z() = rand(mt);
        Eigen::Vector3d p3;
        p3.x() = rand(mt);
        p3.y() = rand(mt);
        p3.z() = rand(mt);
        Eigen::Vector3d p4;
        p4 = p3 + (p2 - p1) * rand(mt);
        if (geometry::IsParallel(p1, p2, p3, p4))
        {
            auto shortest = geometry::ComputeShortestLineBetweenParallelLine3dAndLine3d(p1, p2, p3, p4);
            constexpr double eps = 1.0e-8;
            auto tol = eps * (shortest.second - shortest.first).norm();
            auto dist1 = (shortest.second - shortest.first).norm();
            auto dist2 = geometry::ComputeLengthPoint3dAndLine3d(p1, p3, p4);
            if (std::abs(dist1 - dist2) > tol)
            {
                success = false;
                break;
            }
        }
    }
    EXPECT_TRUE(success);
}
TEST(UnitTest_IntersectionTwoLine3d, SkewRandom)
{
    // all points must be paralell
    bool success = true;
    geometry::PlaneEquation pe;
    {
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<double> rand(-1.0e-0, 1.0e-0);
        pe = geometry::GetPlaneEquation(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)),
                                        Eigen::Vector3d(rand(mt), rand(mt), rand(mt)),
                                        Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
    }
    {
        for (std::size_t i = 0; i < 10000; i++)
        {
            std::random_device rnd;
            std::mt19937 mt(rnd());
            std::uniform_real_distribution<double> rand(-1.0e0, 1.0e0);
            Eigen::Vector3d p1;
            p1.x() = rand(mt);
            p1.y() = rand(mt);
            p1.z() = -1.0e0 * (pe.a_ * p1.x() + pe.b_ * p1.y() + pe.d_) / pe.c_;
            Eigen::Vector3d p2;
            p2.x() = rand(mt);
            p2.y() = rand(mt);
            p2.z() = -1.0e0 * (pe.a_ * p2.x() + pe.b_ * p2.y() + pe.d_) / pe.c_;
            Eigen::Vector3d p3;
            p3.x() = rand(mt);
            p3.y() = rand(mt);
            p3.z() = -1.0e0 * (pe.a_ * p3.x() + pe.b_ * p3.y() + pe.d_) / pe.c_;
            Eigen::Vector3d p4;
            p4.x() = rand(mt);
            p4.y() = rand(mt);
            p4.z() = -1.0e0 * (pe.a_ * p4.x() + pe.b_ * p4.y() + pe.d_) / pe.c_;
            Eigen::Vector3d interesection;
            if (!geometry::HasIntersectionOfLine3dAndLine3d(p1, p2, p3, p4, interesection))
            {
                success = false;
                break;
            } // if
        }
    }
    EXPECT_TRUE(success);
}
TEST(UnitTest_IntersectionTwoLine3d, SkewRandom_SmallerOrder)
{
    // all points must be paralell
    bool success = true;
    geometry::PlaneEquation pe;
    {
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<double> rand(-1.0e-20, 1.0e-20);
        pe = geometry::GetPlaneEquation(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)),
                                        Eigen::Vector3d(rand(mt), rand(mt), rand(mt)),
                                        Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
    }
    {
        for (std::size_t i = 0; i < 10000; i++)
        {
            std::random_device rnd;
            std::mt19937 mt(rnd());
            std::uniform_real_distribution<double> rand(-1.0e-20, 1.0e-20);
            Eigen::Vector3d p1;
            p1.x() = rand(mt);
            p1.y() = rand(mt);
            p1.z() = -1.0e0 * (pe.a_ * p1.x() + pe.b_ * p1.y() + pe.d_) / pe.c_;
            Eigen::Vector3d p2;
            p2.x() = rand(mt);
            p2.y() = rand(mt);
            p2.z() = -1.0e0 * (pe.a_ * p2.x() + pe.b_ * p2.y() + pe.d_) / pe.c_;
            Eigen::Vector3d p3;
            p3.x() = rand(mt);
            p3.y() = rand(mt);
            p3.z() = -1.0e0 * (pe.a_ * p3.x() + pe.b_ * p3.y() + pe.d_) / pe.c_;
            Eigen::Vector3d p4;
            p4.x() = rand(mt);
            p4.y() = rand(mt);
            p4.z() = -1.0e0 * (pe.a_ * p4.x() + pe.b_ * p4.y() + pe.d_) / pe.c_;
            Eigen::Vector3d interesection;
            if (!geometry::HasIntersectionOfLine3dAndLine3d(p1, p2, p3, p4, interesection))
            {
                success = false;
                break;
            } // if
        }
    }
    EXPECT_TRUE(success);
}
TEST(UnitTest_IntersectionTwoLine3d, SkewRandom_LargerOrder)
{
    // all points must be paralell
    bool success = true;
    geometry::PlaneEquation pe;
    {
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<double> rand(-1.0e+20, 1.0e+20);
        pe = geometry::GetPlaneEquation(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)),
                                        Eigen::Vector3d(rand(mt), rand(mt), rand(mt)),
                                        Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
    }
    {
        for (std::size_t i = 0; i < 10000; i++)
        {
            std::random_device rnd;
            std::mt19937 mt(rnd());
            std::uniform_real_distribution<double> rand(-1.0e+20, 1.0e+20);
            Eigen::Vector3d p1;
            p1.x() = rand(mt);
            p1.y() = rand(mt);
            p1.z() = -1.0e0 * (pe.a_ * p1.x() + pe.b_ * p1.y() + pe.d_) / pe.c_;
            Eigen::Vector3d p2;
            p2.x() = rand(mt);
            p2.y() = rand(mt);
            p2.z() = -1.0e0 * (pe.a_ * p2.x() + pe.b_ * p2.y() + pe.d_) / pe.c_;
            Eigen::Vector3d p3;
            p3.x() = rand(mt);
            p3.y() = rand(mt);
            p3.z() = -1.0e0 * (pe.a_ * p3.x() + pe.b_ * p3.y() + pe.d_) / pe.c_;
            Eigen::Vector3d p4;
            p4.x() = rand(mt);
            p4.y() = rand(mt);
            p4.z() = -1.0e0 * (pe.a_ * p4.x() + pe.b_ * p4.y() + pe.d_) / pe.c_;
            Eigen::Vector3d interesection;
            if (!geometry::HasIntersectionOfLine3dAndLine3d(p1, p2, p3, p4, interesection))
            {
                success = false;
                break;
            } // if
        }
    }
    EXPECT_TRUE(success);
}


TEST(UnitTest_IntersectionLine3dAndPlane, CompareResult)
{
    { // ex1
        // http://www.ambrsoft.com/TrigoCalc/Plan3D/PlaneLineIntersection_.htm
        geometry::PlaneEquation pe;
        pe.a_ = 3.0e0;
        pe.b_ = -1.0e0;
        pe.c_ = 2.0e0;
        pe.d_ = -5.0e0;
        Eigen::Vector3d intersection;
        auto result = HasIntersectionLine3dAndPlane(Eigen::Vector3d(1.0e0, -1.0e0, 1.0e0),
                                                    Eigen::Vector3d(-1.0e0, -4.0e0, 3.0e0),
                                                    pe, intersection);
        EXPECT_TRUE(result);
        EXPECT_DOUBLE_EQ(intersection.x(), 3.0e0);
        EXPECT_DOUBLE_EQ(intersection.y(), 2.0e0);
        EXPECT_DOUBLE_EQ(intersection.z(), -1.0e0);
    }
    { // ex2
        // https://www.geisya.or.jp/~mwm48961/linear_algebra/line_plane3.htm
        // ex 1
        geometry::PlaneEquation pe;
        pe.a_ = 1.0e0;
        pe.b_ = -2.0e0;
        pe.c_ = 3.0e0;
        pe.d_ = 10.0e0;
        Eigen::Vector3d intersection;
        auto result = HasIntersectionLine3dAndPlane(Eigen::Vector3d(1.0e0, 1.0e0, -4.0e0),
                                                    Eigen::Vector3d(-5.0e0, 3.0e0, 0.0e0),
                                                    pe, intersection);
        EXPECT_DOUBLE_EQ(intersection.x(), -8.0e0);
        EXPECT_DOUBLE_EQ(intersection.y(), 4.0e0);
        EXPECT_NEAR(intersection.z() - 2.0e0, 0.0e0);
    }
}
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
