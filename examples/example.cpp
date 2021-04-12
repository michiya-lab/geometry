#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <set>
#include <random>

#include <Eigen/Dense>

#include <geometry.hpp>
#include <point3d.hpp>

namespace
{
    bool IsParallel(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                    const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                    const double &eps = 1.0e-8)
    {
        return (((p2 - p1).normalized()).cross((p4 - p3).normalized())).norm() < eps ? true : false;
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeShortestLineBetweenSkewLine3dAndLine3d(
        const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
        const double &eps = 1.0e-8)
    {
    }
    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeShortestLineBetweenParallelLine3dAndLine3d(
        const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
        const double &eps = 1.0e-8)
    {

    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeShortestLine3dBetweenLine3dAndLine3d(
        const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
        const double &eps = 1.0e-8)
    {
        // http://paulbourke.net/geometry/pointlineplane/
        std::pair<Eigen::Vector3d, Eigen::Vector3d> shortest_line;
        shortest_line.first = Eigen::Vector3d(-1.0e8, -1.0e8, -1.0e8);
        shortest_line.second = Eigen::Vector3d(1.0e8, 1.0e8, 1.0e8);
        //if ((((p2 - p1).normalized()).cross((p4 - p3).normalized())).norm() < eps)
        if (IsParallel(p1, p2, p3, p4, eps))
            return shortest_line;
        auto v21 = p2 - p1;
        auto v43 = p4 - p3;
        auto v13 = p1 - p3;
        double tol = eps * std::max(v21.norm(), v43.norm());
        if (v21.norm() < tol)
            return shortest_line;
        if (v43.norm() < tol)
            return shortest_line;
        auto d1321 = v13.dot(v21);
        auto d1343 = v13.dot(v43);
        auto d2121 = v21.dot(v21);
        auto d4321 = v43.dot(v21);
        auto d4343 = v43.dot(v43);
        if (std::abs(d2121 * d4343 - d4321 * d4321) < tol)
            return shortest_line;
        double mu_a = (d1343 * d4321 - d1321 * d4343) / (d2121 * d4343 - d4321 * d4321);
        double mu_b = (d1343 + mu_a * d4321) / d4343;
        shortest_line.first = p1 + mu_a * (p2 - p1);
        shortest_line.second = p3 + mu_b * (p4 - p3);
        return shortest_line;
    }
    double ComputeShortestLengthLine3dAndLine3d(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                                const Eigen::Vector3d &p3, const Eigen::Vector3d &p4)
    {
        auto shortest_line = ComputeShortestLine3dBetweenLine3dAndLine3d(p1, p2, p3, p4);
        return (shortest_line.second - shortest_line.first).norm();
    }
    bool hasIntersectionOfLine3dAndLine3d(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                          const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                                          Eigen::Vector3d& intersection,
                                          const double &eps = 1.0e-8)
    {
        auto shortest_line = ComputeShortestLine3dBetweenLine3dAndLine3d(p1, p2, p3, p4, eps);
        intersection = shortest_line.first;
        auto length_1 = std::max((p1 - shortest_line.first).norm(), (p2 - shortest_line.first).norm());
        auto length_2 = std::max((p3 - shortest_line.second).norm(), (p4 - shortest_line.second).norm());
        double tol = eps * std::max(length_1, length_2);
        if ((shortest_line.second - shortest_line.first).norm() < tol)
        {
            return true;
        } // if
        return false;
    }

    bool IsOnLineSegment(const Eigen::Vector3d &query, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                         const double &eps = 1.0e-8)
    {
        auto length_12 = (p2 - p1).norm();
        auto length_q1 = (p1 - query).norm();
        auto length_q2 = (p2 - query).norm();
        auto tol = eps * length_12;
        if (length_q1 + length_q2 <= length_12 + tol)
        {
            return true;
        } // ilength_q1+length_q2=< f length_12+eps
        return false;
    }

    class PlaneEquation
    {
    public:
        PlaneEquation() { ; }
        virtual ~PlaneEquation() { ; }
        double a_ = 0.0e0;
        double b_ = 0.0e0;
        double c_ = 0.0e0;
        double d_ = 0.0e0;

    public:                                                        // oparator
        PlaneEquation(const PlaneEquation &) = default;            // copy constractor
        PlaneEquation(PlaneEquation &&) = default;                 // move constractor
        PlaneEquation &operator=(const PlaneEquation &) = default; // copy assignment operator
        PlaneEquation &operator=(PlaneEquation &&) = default;      // move assignment operator
    };                                                             // class PlaneEquation
} // namespace

int main()
{
    auto dist = ComputeShortestLengthLine3dAndLine3d(Eigen::Vector3d(-1.0e0, 2.0e0, 0.0e0),
                                                     Eigen::Vector3d(1.0e0, 5.0e0, 1.0e0),
                                                     Eigen::Vector3d(3.0e0, -4.0e0, 1.0e0),
                                                     Eigen::Vector3d(4.0e0, -2.0e0, 2.0e0));
    std::cout << dist << std::endl;
    Eigen::Vector3d interesection;
    std::cout << hasIntersectionOfLine3dAndLine3d(Eigen::Vector3d(0.0e0, 9.0e0, 0.0e0),
                                                  Eigen::Vector3d(1.0e0, 11.0e0, 0.0e0),
                                                  Eigen::Vector3d(0.0e0, 7.0e0, 1.0e-8),
                                                  Eigen::Vector3d(1.0e0, 10.0e0, 1.0e-8),
                                                  interesection,
                                                  1.0e-4)
              << std::endl;
    std::cout << interesection << std::endl;
    { // plane
        PlaneEquation pe;
        {
            std::random_device rnd;
            std::mt19937 mt(rnd());
            std::uniform_real_distribution<double> rand(-1.0e5, 1.0e5);
            pe.a_ = rand(mt);
            pe.b_ = rand(mt);
            pe.c_ = rand(mt);
            pe.d_ = rand(mt);
        }
        {
            for (std::size_t i = 0; i < 1000 * 1000; i++)
            {
                std::random_device rnd;
                std::mt19937 mt(rnd());
                std::uniform_real_distribution<double> rand(-1.0e5, 1.0e5);
                Eigen::Vector3d p1;
                p1.x() = rand(mt);
                p1.y() = rand(mt);
                p1.z() = (pe.a_ * p1.x() + pe.b_ * p1.y() + pe.d_) / pe.c_;
                Eigen::Vector3d p2;
                p2.x() = rand(mt);
                p2.y() = rand(mt);
                p2.z() = (pe.a_ * p2.x() + pe.b_ * p2.y() + pe.d_) / pe.c_;
                Eigen::Vector3d p3;
                p3.x() = rand(mt);
                p3.y() = rand(mt);
                p3.z() = (pe.a_ * p3.x() + pe.b_ * p3.y() + pe.d_) / pe.c_;
                Eigen::Vector3d p4;
                p4.x() = rand(mt);
                p4.y() = rand(mt);
                p4.z() = (pe.a_ * p4.x() + pe.b_ * p4.y() + pe.d_) / pe.c_;
                Eigen::Vector3d interesection;
                if (!hasIntersectionOfLine3dAndLine3d(p1, p2, p3, p4, interesection))
                {
                    std::cout << "error" << std::endl;
                    std::cout << ComputeShortestLengthLine3dAndLine3d(p1, p2, p3, p4) << std::endl;
                    // std::cout << pe.a_ * p1.x() + pe.b_ * p1.y() - pe.c_ * p1.z() + pe.d_ << std::endl;
                    // std::cout << pe.a_ * p2.x() + pe.b_ * p2.y() - pe.c_ * p2.z() + pe.d_ << std::endl;
                    // std::cout << pe.a_ * p3.x() + pe.b_ * p3.y() - pe.c_ * p3.z() + pe.d_ << std::endl;
                    // std::cout << pe.a_ * p4.x() + pe.b_ * p4.y() - pe.c_ * p4.z() + pe.d_ << std::endl;
                    std::cout << "p1" << std::endl;
                    std::cout << std::setprecision(13) << p1.x() << ", " << p1.y() << ", " << p1.z() << std::endl;
                    std::cout << "p2" << std::endl;
                    std::cout << std::setprecision(13) << p2.x() << ", " << p2.y() << ", " << p2.z() << std::endl;
                    std::cout << "p3" << std::endl;
                    std::cout << std::setprecision(13) << p3.x() << ", " << p3.y() << ", " << p3.z() << std::endl;
                    std::cout << "p4" << std::endl;
                    std::cout << std::setprecision(13) << p4.x() << ", " << p4.y() << ", " << p4.z() << std::endl;
                    std::cout << "vector" << std::endl;
                    std::cout << std::setprecision(13) << (p2 - p1) << std::endl;
                    std::cout << std::setprecision(13) << (p4 - p3) << std::endl;
                    std::cout << "vector (normalized)" << std::endl;
                    std::cout << std::setprecision(13) << (p2 - p1).normalized() << std::endl;
                    std::cout << std::setprecision(13) << (p4 - p3).normalized() << std::endl;
                    std::cout << "norm" << std::endl;
                    std::cout << ((p2 - p1).cross(p4 - p3)) << std::endl;
                    std::cout << ((p2 - p1).cross(p4 - p3)).norm() << std::endl;
                    std::cout << (((p2 - p1).normalized()).cross((p4 - p3).normalized())).norm() << std::endl;
                    // std::cout << interesection << std::endl;
                } // if
                if (IsOnLineSegment(interesection, p1, p2) && IsOnLineSegment(interesection, p3, p4))
                {
                    std::cout << "" << std::endl;
                    std::cout << std::setprecision(13) << p1.x() << " " << p1.y() << " " << p1.z() << std::endl;
                    std::cout << std::setprecision(13) << p2.x() << " " << p2.y() << " " << p2.z() << std::endl;
                    std::cout << std::setprecision(13) << p3.x() << " " << p3.y() << " " << p3.z() << std::endl;
                    std::cout << std::setprecision(13) << p4.x() << " " << p4.y() << " " << p4.z() << std::endl;
                }
            }
        }
    }
    return 0;
}
