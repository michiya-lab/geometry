#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <set>
#include <random>
#include <vector>

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

        // http://paulbourke.net/geometry/pointlineplane/
        std::pair<Eigen::Vector3d, Eigen::Vector3d> shortest_line;
        shortest_line.first = Eigen::Vector3d(-1.0e8, -1.0e8, -1.0e8);
        shortest_line.second = Eigen::Vector3d(1.0e8, 1.0e8, 1.0e8);
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
    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeShortestLineBetweenParallelLine3dAndLine3d(
        const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
        const double &eps = 1.0e-8)
    {
        auto v21 = p2 - p1;
        auto v43 = p4 - p3;
        double tol = eps * std::max(v21.norm(), v43.norm());
        if (v21.norm() > tol)
        {
            std::pair<Eigen::Vector3d, Eigen::Vector3d> shortest_line;
            shortest_line.first = Eigen::Vector3d(-1.0e8, -1.0e8, -1.0e8);
            shortest_line.second = Eigen::Vector3d(1.0e8, 1.0e8, 1.0e8);
            auto v13 = p1 - p3;
            auto d1343 = v13.dot(v43);
            shortest_line.first = p1;
            shortest_line.second = p3 + v43.normalized() * d1343 / v43.norm();
            return shortest_line;
        } // if v21.norm() > tol
        else if (v43.norm() > tol)
        {
            std::pair<Eigen::Vector3d, Eigen::Vector3d> shortest_line;
            shortest_line.first = Eigen::Vector3d(-1.0e8, -1.0e8, -1.0e8);
            shortest_line.second = Eigen::Vector3d(1.0e8, 1.0e8, 1.0e8);
            auto v31 = p3 - p1;
            auto d3121 = v31.dot(v21);
            shortest_line.first = p3;
            shortest_line.second = p1 + v21.normalized() * d3121 / v21.norm();
            return shortest_line;
        } // else if v43.norm() >tol
        else
        {
            std::pair<Eigen::Vector3d, Eigen::Vector3d> shortest_line;
            shortest_line.first = p1;
            shortest_line.second = p3;
            return shortest_line;
        }
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeShortestLine3dBetweenLine3dAndLine3d(
        const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
        const double &eps = 1.0e-8)
    {
        //if ((((p2 - p1).normalized()).cross((p4 - p3).normalized())).norm() < eps)
        if (IsParallel(p1, p2, p3, p4, eps))
        {
            return ComputeShortestLineBetweenParallelLine3dAndLine3d(p1, p2, p3, p4, eps);
        }
        else
        {
            return ComputeShortestLineBetweenSkewLine3dAndLine3d(p1, p2, p3, p4, eps);
        }
    }
    double ComputeShortestLengthLine3dAndLine3d(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                                const Eigen::Vector3d &p3, const Eigen::Vector3d &p4)
    {
        auto shortest_line = ComputeShortestLine3dBetweenLine3dAndLine3d(p1, p2, p3, p4);
        return (shortest_line.second - shortest_line.first).norm();
    }
    double ComputeLengthPoint3dAndLine3d(const Eigen::Vector3d &query, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                         const double &eps = 1.0e-8)
    {
        // https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
        auto length_21 = (p2 - p1).norm();
        if (length_21 < eps)
        {
            return (query - p1).norm();
        } // if
        return ((p2 - p1).cross(p1 - query)).norm() / length_21;
    }
    bool hasIntersectionOfLine3dAndLine3d(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                          const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                                          Eigen::Vector3d &intersection,
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
    bool hasIntersectionOfLine3dSegmentAndLine3dSegment(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                                        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                                                        Eigen::Vector3d &intersection,
                                                        const double &eps = 1.0e-8)
    {
        if (hasIntersectionOfLine3dAndLine3d(p1, p2, p3, p4, intersection, eps))
        {
            return (IsOnLineSegment(intersection, p1, p2) && IsOnLineSegment(intersection, p3, p4)) ? true : false;
        }
        else
        {
            return false;
        }
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

    PlaneEquation GetPlaneEquation(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3)
    {
        auto x1 = p1.x();
        auto y1 = p1.y();
        auto z1 = p1.z();
        auto x2 = p2.x();
        auto y2 = p2.y();
        auto z2 = p2.z();
        auto x3 = p3.x();
        auto y3 = p3.y();
        auto z3 = p3.z();
        PlaneEquation pe;
        pe.a_ = (y2 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1);
        pe.b_ = (z2 - z1) * (x3 - x1) - (z3 - z1) * (x2 - x1);
        pe.c_ = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
        pe.d_ = -1.0e0 * (pe.a_ * x1 + pe.b_ * y1 + pe.c_ * z1);
        return pe;
    }
    bool IsOnPlane(const Eigen::Vector3d &query, const PlaneEquation &plane_equation,
                   const double &eps = 1.0e-8)
    {
        return (plane_equation.a_ * query.x() + plane_equation.b_ * query.y() + plane_equation.c_ * query.z() + plane_equation.d_ < eps) ? true : false;
    }
    bool IsInsidePolygon3d(const Eigen::Vector3d &query, const std::vector<Eigen::Vector3d> &points,
                           const double &eps = 1.0e-8)
    {
        if (points.size() < 3)
            return false;
        { // check points on same plane
            auto pe = GetPlaneEquation(points[0], points[1], points[2]);
            for (std::size_t i = 3; i < points.size(); ++i)
            {
                if (!IsOnPlane(points[i], pe, eps))
                    return false;
            } // for i
        }
        { // check query is on edge
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                if (i == points.size() - 1)
                {
                    if (IsOnLineSegment(query, points[i], points[0]))
                        return true;
                }
                else
                {
                    if (IsOnLineSegment(query, points[i], points[i + 1]))
                        return true;
                }
            }
        }
        { // ray casting algorithm
            double squered_dist = 0.0e0;
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                squered_dist = std::max(squered_dist, (points[i] - query).squaredNorm());
            } // for i
            auto dist = std::sqrt(squered_dist);
            auto ray_length = dist * 2.0e0;
            long num_of_try = 0;
            long num_of_intersection = 0;
            { // find intersection
                while (true)
                {
                    num_of_intersection = 0;
                    bool try_other_direction = false;
                    Eigen::Vector3d direction_vector = Eigen::Vector3d(1.0e0, 1.0e0, 1.0e0);
                    { // select direction vector
                        if (num_of_try == 0)
                        {
                            direction_vector = Eigen::Vector3d(1.0e0, 1.0e-2, 1.0e-2); // almost parallel to x-axis
                        }                                                              // if
                        else if (num_of_try == 1)
                        {
                            direction_vector = Eigen::Vector3d(1.0e-2, 1.0e0, 1.0e-2); // almost parallel to y-axis
                        }                                                              // else if num_of
                        else if (num_of_try == 2)
                        {
                            direction_vector = Eigen::Vector3d(1.0e-2, 1.0e-2, 1.0e0); // almost parallel to z-axis
                        }                                                              // else if num_of
                        else if (num_of_try < 100)
                        {
                            std::random_device rnd;
                            std::mt19937 mt(rnd());
                            std::uniform_real_distribution<double> rand(0, 1.0e0);
                            direction_vector = Eigen::Vector3d(rand(mt), rand(mt), rand(mt));
                        } // else if
                        else
                        {
                            std::cout << "FATAL ERROR" << std::endl;
                            exit(1);
                            // break; // here, give up to find good direction vector
                        }
                        direction_vector.normalize();
                    }
                    auto ray_point = query + direction_vector * ray_length;
                    Eigen::Vector3d intersection;
                    for (std::size_t i = 0; i < points.size(); ++i)
                    {
                        Eigen::Vector3d edge_p1, edge_p2;
                        if (i == points.size() - 1)
                        {
                            edge_p1 = points[i];
                            edge_p2 = points[0];
                        }
                        else
                        {
                            edge_p1 = points[i];
                            edge_p2 = points[i + 1];
                        }
                        if (hasIntersectionOfLine3dSegmentAndLine3dSegment(query, ray_point, edge_p1, edge_p2, intersection))
                        {
                            auto tol = eps * (edge_p2 - edge_p1).norm();
                            if ((intersection - edge_p2).squaredNorm() < tol * tol)
                            {
                                try_other_direction = true;
                                break;
                            }
                            if ((intersection - edge_p1).squaredNorm() < tol * tol)
                            {
                                try_other_direction = true;
                                break;
                            }
                            num_of_intersection++;
                        }
                    }
                    if (!try_other_direction)
                        break;

                }
            }
            std::cout << "num of intersection " << num_of_intersection << std::endl;
            if (num_of_intersection % 2 == 0)
            {
                return false;
            } // if num_
            else
            {
                return true;
            }
        }
    }
} // namespace

int main()
{
    {
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<double> rand(-1.0e2, 1.0e2);
        std::vector<Eigen::Vector3d> points;
        points.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
        points.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
        points.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
        std::cout << std::setprecision(13)
                  << points[0].x() << " "
                  << points[0].y() << " "
                  << points[0].z() << std::endl;
        std::cout << std::setprecision(13)
                  << points[1].x() << " "
                  << points[1].y() << " "
                  << points[1].z() << std::endl;
        std::cout << std::setprecision(13)
                  << points[2].x() << " "
                  << points[2].y() << " "
                  << points[2].z() << std::endl;
        std::cout << std::endl;
        auto pe = GetPlaneEquation(points[0], points[1], points[2]);
        std::vector<Eigen::Vector3d> queries;
        std::size_t count = 0;
        for (std::size_t i = 0; i < 10000; ++i)
        {
            Eigen::Vector3d p;
            p.x() = rand(mt);
            p.y() = rand(mt);
            p.z() = -1.0e0 * (pe.a_ * p.x() + pe.b_ * p.y() + pe.d_) / pe.c_;
            queries.push_back(p);
            if (IsInsidePolygon3d(p, points))
            {
                count++;
            }
        } // for i
        std::cout << "# vtk DataFile Version 2.0" << std::endl
                  << "points" << std::endl
                  << "ASCII" << std::endl
                  << "DATASET UNSTRUCTURED_GRID " << std::endl
                  << "POINTS " << count << " double"
                  << std::endl;
        for (std::size_t i = 0; i < queries.size(); ++i)
        {
            if (IsInsidePolygon3d(queries[i], points))
            {
                std::cout << std::setprecision(13)
                          << queries[i].x() << "  "
                          << queries[i].y() << "  "
                          << queries[i].z() << std::endl;
            }
        } // for i
        std::cout << "CELLS " << count << " " << count * 2 << std::endl;
        for (std::size_t i = 0; i < count; ++i)
        {
            std::cout << "1 " << i << " ";
        } // for i
        std::cout << std::endl;
        std::cout << "CELL_TYPES " << count << std::endl;
        for (std::size_t i = 0; i < count; ++i)
        {
            std::cout << "1 ";
        } // for i
        std::cout << std::endl;
    }
    return 0;
    { // test1
        auto p1 = Eigen::Vector3d(1.0e0, 1.0e0, 0.0e0);
        auto p2 = Eigen::Vector3d(2.0e0, 2.0e0, 0.0e0);
        auto p3 = Eigen::Vector3d(-1.0e0, -1.0e0, 0.0e0);
        auto p4 = Eigen::Vector3d(4.0e0, 4.0e0, 0.0e0);
        auto shortest = ComputeShortestLineBetweenParallelLine3dAndLine3d(p1, p2, p3, p4);
        std::cout << (shortest.second - shortest.first).norm() << std::endl;
        std::cout << ComputeLengthPoint3dAndLine3d(p1, p3, p4) << std::endl;
    }
    { // test1
        auto p1 = Eigen::Vector3d(-1.0e0, 2.0e0, 0.0e0);
        auto p2 = Eigen::Vector3d(1.0e0, 5.0e0, 1.0e0);
        auto p3 = Eigen::Vector3d(3.0e0, -4.0e0, 1.0e0);
        auto p4 = Eigen::Vector3d(5.0e0, -1.0e0, 2.0e0);
        auto shortest = ComputeShortestLineBetweenParallelLine3dAndLine3d(p1, p2, p3, p4);
        std::cout << (shortest.second - shortest.first).norm() << std::endl;
        std::cout << ComputeLengthPoint3dAndLine3d(p1, p3, p4) << std::endl;
    }
    { // test no intersection for skew lines
        // https://www.mathepower.com/en/lineintersection.php
        auto p1 = Eigen::Vector3d(2.0e0, 3.0e0, 4.0e0);
        auto p2 = Eigen::Vector3d(8.0e0, -1.0e0, 5.0e0);
        auto p3 = Eigen::Vector3d(-3.0e0, -1.0e0, 8.0e0);
        auto p4 = Eigen::Vector3d(10.0e0, -8.0e0, 7.0e0);
        Eigen::Vector3d intersection;
        if (hasIntersectionOfLine3dSegmentAndLine3dSegment(p1, p2, p3, p4, intersection) == false)
        {
            std::cout << "example 2 is ok" << std::endl;
        }
    }
    {
        // https://math.stackexchange.com/questions/270767/find-intersection-of-two-3d-lines/271366
        auto p1 = Eigen::Vector3d(5.0e0, 5.0e0, 4.0e0);
        auto p2 = Eigen::Vector3d(10.0e0, 10.0e0, 6.0e0);
        auto p3 = Eigen::Vector3d(5.0e0, 5.0e0, 5.0e0);
        auto p4 = Eigen::Vector3d(10.0e0, 10.0e0, 3.0e0);
        Eigen::Vector3d intersection;
        if (hasIntersectionOfLine3dSegmentAndLine3dSegment(p1, p2, p3, p4, intersection) == true)
        {
            auto ans = Eigen::Vector3d(6.25e0, 6.25e0, 4.5e0);
            if ((ans - intersection).norm() < 1.0e-8)
            {
                std::cout << "example 3 is ok" << std::endl;
            }
        }
    }
    { // parallel line
        for (std::size_t i = 0; i < 1000 * 100; i++)
        {
            std::random_device rnd;
            std::mt19937 mt(rnd());
            std::uniform_real_distribution<double> rand(-1.0e5, 1.0e5);
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
            if (IsParallel(p1, p2, p3, p4))
            {
                auto shortest = ComputeShortestLineBetweenParallelLine3dAndLine3d(p1, p2, p3, p4);
                constexpr double eps = 1.0e-8;
                auto tol = eps * (shortest.second - shortest.first).norm();
                auto dist1 = (shortest.second - shortest.first).norm();
                auto dist2 = ComputeLengthPoint3dAndLine3d(p1, p3, p4);
                if (std::abs(dist1 - dist2) > tol)
                {
                    std::cout << "error" << std::endl;
                }
            }
        }
    }
    return 0;
    { // plane for skew line
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
            }
        }
    }
    return 0;
}
