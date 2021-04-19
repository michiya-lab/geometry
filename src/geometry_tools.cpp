#include <geometry_tools.hpp>

namespace geometry
{
    bool IsParallel(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                    const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                    const double &eps)
    {
        return (((p2 - p1).normalized()).cross((p4 - p3).normalized())).norm() < eps ? true : false;
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeShortestLineBetweenSkewLine3dAndLine3d(
        const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
        const double &eps)
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
        if (std::abs(d2121 * d4343 - d4321 * d4321) < tol * tol * tol * tol)
        {
            return shortest_line;
        }
        double mu_a = (d1343 * d4321 - d1321 * d4343) / (d2121 * d4343 - d4321 * d4321);
        double mu_b = (d1343 + mu_a * d4321) / d4343;
        shortest_line.first = p1 + mu_a * (p2 - p1);
        shortest_line.second = p3 + mu_b * (p4 - p3);
        return shortest_line;
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeShortestLineBetweenParallelLine3dAndLine3d(
        const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
        const double &eps)
    {
        auto v21 = p2 - p1;
        auto v43 = p4 - p3;
        double tol = eps * std::max(v21.norm(), v43.norm());
        if (v43.norm() > tol)
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
        else if (v21.norm() > tol)
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
        const double &eps)
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
                                         const double &eps)
    {
        // https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
        auto length_21 = (p2 - p1).norm();
        auto tol = std::max({std::abs(p2.x() - p1.x()), std::abs(p2.y() - p1.y()), std::abs(p2.z() - p1.z())}) * eps;
        if (length_21 < tol)
        {
            return (query - p1).norm();
        } // if
        return ((p2 - p1).cross(p1 - query)).norm() / length_21;
    }

    bool HasIntersectionOfLine3dAndLine3d(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                          const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                                          Eigen::Vector3d &intersection,
                                          const double &eps)
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
                         const double &eps)
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

    bool HasIntersectionOfLine3dSegmentAndLine3dSegment(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                                        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                                                        Eigen::Vector3d &intersection,
                                                        const double &eps)
    {
        if (HasIntersectionOfLine3dAndLine3d(p1, p2, p3, p4, intersection, eps))
        {
            return (IsOnLineSegment(intersection, p1, p2) && IsOnLineSegment(intersection, p3, p4)) ? true : false;
        }
        else
        {
            return false;
        }
    }

    PlaneEquation GetPlaneEquation(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3)
    {
        auto v21 = p2 - p1;
        auto v31 = p3 - p1;
        auto cross = (v31.cross(v21)).normalized();
        PlaneEquation pe;
        pe.a_ = cross.x();
        pe.b_ = cross.y();
        pe.c_ = cross.z();
        pe.d_ = -1.0e0 * (pe.a_ * p1.x() + pe.b_ * p1.y() + pe.c_ * p1.z());
        return pe;
    }

    bool IsOnPlane(const Eigen::Vector3d &query, const PlaneEquation &plane_equation, const double &eps)
    {
        return (std::abs(plane_equation.a_ * query.x() + plane_equation.b_ * query.y() + plane_equation.c_ * query.z() + plane_equation.d_) < eps) ? true : false;
    }

    bool IsInsidePolygon3dRayCastingAlgorithm(const Eigen::Vector3d &query, const std::vector<Eigen::Vector3d> &points, const double &eps)
    {
        if (points.size() < 3)
            return false;
        { // check points on same plane
            auto pe = GetPlaneEquation(points[0], points[1], points[2]);
            for (std::size_t i = 3; i < points.size(); ++i)
            {
                if (!IsOnPlane(points[i], pe, eps))
                {
                    return false;
                }
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
            auto ray_length = dist * 10.0e0;
            long num_of_try = 0;
            long num_of_intersection = 0;
            { // find intersection
                while (true)
                {
                    num_of_intersection = 0;
                    bool try_other_direction = false;
                    Eigen::Vector3d direction_vector;
                    { // select direction vector
                        auto pe = GetPlaneEquation(points[0], points[1], points[2]);
                        if (std::abs(pe.c_) > eps)
                        {
                            if (num_of_try == 0)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.b_ * temp_p1.y() + pe.d_) / pe.c_;
                                temp_p2.x() = 1.0e0;
                                temp_p2.y() = 1.0e-3;
                                temp_p2.z() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.b_ * temp_p2.y() + pe.d_) / pe.c_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try == 1)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.b_ * temp_p1.y() + pe.d_) / pe.c_;
                                temp_p2.x() = 1.0e-3;
                                temp_p2.y() = 1.0e0;
                                temp_p2.z() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.b_ * temp_p2.y() + pe.d_) / pe.c_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try < 100)
                            {
                                std::random_device rnd;
                                std::mt19937 mt(rnd());
                                std::uniform_real_distribution<double> rand(0, 1.0e0);
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.b_ * temp_p1.y() + pe.d_) / pe.c_;
                                temp_p2.x() = rand(mt);
                                temp_p2.y() = rand(mt);
                                temp_p2.z() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.b_ * temp_p2.y() + pe.d_) / pe.c_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else
                            {
                                return false;
                            }
                        } // if pe.a_<tol
                        else if (std::abs(pe.b_) > eps)
                        {
                            if (num_of_try == 0)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.y() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.c_ * temp_p1.z() + pe.d_) / pe.b_;
                                temp_p2.x() = 1.0e0;
                                temp_p1.z() = 1.0e-3;
                                temp_p2.y() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.c_ * temp_p2.z() + pe.d_) / pe.b_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try == 1)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.y() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.c_ * temp_p1.z() + pe.d_) / pe.b_;
                                temp_p2.x() = 1.0e-3;
                                temp_p1.z() = 1.0e0;
                                temp_p2.y() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.c_ * temp_p2.z() + pe.d_) / pe.b_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try < 100)
                            {
                                std::random_device rnd;
                                std::mt19937 mt(rnd());
                                std::uniform_real_distribution<double> rand(0, 1.0e0);
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.y() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.c_ * temp_p1.z() + pe.d_) / pe.b_;
                                temp_p2.x() = rand(mt);
                                temp_p1.z() = rand(mt);
                                temp_p2.y() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.c_ * temp_p2.z() + pe.d_) / pe.b_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else
                            {
                                return false;
                            }
                        } // else if
                        else if (std::abs(pe.a_) > eps)
                        {
                            if (num_of_try == 0)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.x() = -1.0e0 * (pe.b_ * temp_p1.y() + pe.c_ * temp_p1.z() + pe.d_) / pe.a_;
                                temp_p2.y() = 1.0e0;
                                temp_p1.z() = 1.0e-3;
                                temp_p2.x() = -1.0e0 * (pe.b_ * temp_p2.y() + pe.c_ * temp_p2.z() + pe.d_) / pe.a_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try == 1)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.x() = -1.0e0 * (pe.b_ * temp_p1.y() + pe.c_ * temp_p1.z() + pe.d_) / pe.a_;
                                temp_p2.y() = 1.0e-3;
                                temp_p1.z() = 1.0e0;
                                temp_p2.x() = -1.0e0 * (pe.b_ * temp_p2.y() + pe.c_ * temp_p2.z() + pe.d_) / pe.a_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try < 100)
                            {
                                std::random_device rnd;
                                std::mt19937 mt(rnd());
                                std::uniform_real_distribution<double> rand(0, 1.0e0);
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.x() = -1.0e0 * (pe.b_ * temp_p1.y() + pe.c_ * temp_p1.z() + pe.d_) / pe.a_;
                                temp_p2.y() = rand(mt);
                                temp_p1.z() = rand(mt);
                                temp_p2.x() = -1.0e0 * (pe.b_ * temp_p2.y() + pe.c_ * temp_p2.z() + pe.d_) / pe.a_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else
                            {
                                return false;
                            }
                        }
                        else
                        {
                            return false;
                        }
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
                        if (HasIntersectionOfLine3dSegmentAndLine3dSegment(query, ray_point, edge_p1, edge_p2, intersection))
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

    bool IsInsidePolygon3dWindingNumberAlgorithm(const Eigen::Vector3d &query, const std::vector<Eigen::Vector3d> &points, const double &eps)
    {
        if (points.size() < 3)
            return false;
        { // check points on same plane
            auto pe = GetPlaneEquation(points[0], points[1], points[2]);
            for (std::size_t i = 3; i < points.size(); ++i)
            {
                if (!IsOnPlane(points[i], pe, eps))
                {
                    return false;
                }
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
        { // winding number algorithm
            double squered_dist = 0.0e0;
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                squered_dist = std::max(squered_dist, (points[i] - query).squaredNorm());
            } // for i
            auto dist = std::sqrt(squered_dist);
            auto ray_length = dist * 10.0e0;
            long num_of_try = 0;
            long num_of_winding = 0;
            auto ref_cross = ((points[2] - points[0]).normalized()).cross((points[1] - points[0]).normalized());
            { // find intersection
                while (true)
                {
                    num_of_winding = 0;
                    bool try_other_direction = false;
                    Eigen::Vector3d direction_vector;
                    { // select direction vector
                        auto pe = GetPlaneEquation(points[0], points[1], points[2]);
                        if (std::abs(pe.c_) > eps)
                        {
                            if (num_of_try == 0)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.b_ * temp_p1.y() + pe.d_) / pe.c_;
                                temp_p2.x() = 1.0e0;
                                temp_p2.y() = 1.0e-3;
                                temp_p2.z() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.b_ * temp_p2.y() + pe.d_) / pe.c_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try == 1)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.b_ * temp_p1.y() + pe.d_) / pe.c_;
                                temp_p2.x() = 1.0e-3;
                                temp_p2.y() = 1.0e0;
                                temp_p2.z() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.b_ * temp_p2.y() + pe.d_) / pe.c_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try < 100)
                            {
                                std::random_device rnd;
                                std::mt19937 mt(rnd());
                                std::uniform_real_distribution<double> rand(0, 1.0e0);
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.b_ * temp_p1.y() + pe.d_) / pe.c_;
                                temp_p2.x() = rand(mt);
                                temp_p2.y() = rand(mt);
                                temp_p2.z() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.b_ * temp_p2.y() + pe.d_) / pe.c_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else
                            {
                                return false;
                            }
                        } // if pe.a_<tol
                        else if (std::abs(pe.b_) > eps)
                        {
                            if (num_of_try == 0)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.y() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.c_ * temp_p1.z() + pe.d_) / pe.b_;
                                temp_p2.x() = 1.0e0;
                                temp_p1.z() = 1.0e-3;
                                temp_p2.y() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.c_ * temp_p2.z() + pe.d_) / pe.b_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try == 1)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.y() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.c_ * temp_p1.z() + pe.d_) / pe.b_;
                                temp_p2.x() = 1.0e-3;
                                temp_p1.z() = 1.0e0;
                                temp_p2.y() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.c_ * temp_p2.z() + pe.d_) / pe.b_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try < 100)
                            {
                                std::random_device rnd;
                                std::mt19937 mt(rnd());
                                std::uniform_real_distribution<double> rand(0, 1.0e0);
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.x() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.y() = -1.0e0 * (pe.a_ * temp_p1.x() + pe.c_ * temp_p1.z() + pe.d_) / pe.b_;
                                temp_p2.x() = rand(mt);
                                temp_p1.z() = rand(mt);
                                temp_p2.y() = -1.0e0 * (pe.a_ * temp_p2.x() + pe.c_ * temp_p2.z() + pe.d_) / pe.b_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else
                            {
                                return false;
                            }
                        } // else if
                        else if (std::abs(pe.a_) > eps)
                        {
                            if (num_of_try == 0)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.x() = -1.0e0 * (pe.b_ * temp_p1.y() + pe.c_ * temp_p1.z() + pe.d_) / pe.a_;
                                temp_p2.y() = 1.0e0;
                                temp_p1.z() = 1.0e-3;
                                temp_p2.x() = -1.0e0 * (pe.b_ * temp_p2.y() + pe.c_ * temp_p2.z() + pe.d_) / pe.a_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try == 1)
                            {
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.x() = -1.0e0 * (pe.b_ * temp_p1.y() + pe.c_ * temp_p1.z() + pe.d_) / pe.a_;
                                temp_p2.y() = 1.0e-3;
                                temp_p1.z() = 1.0e0;
                                temp_p2.x() = -1.0e0 * (pe.b_ * temp_p2.y() + pe.c_ * temp_p2.z() + pe.d_) / pe.a_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else if (num_of_try < 100)
                            {
                                std::random_device rnd;
                                std::mt19937 mt(rnd());
                                std::uniform_real_distribution<double> rand(0, 1.0e0);
                                Eigen::Vector3d temp_p1, temp_p2;
                                temp_p1.y() = 0.0e0;
                                temp_p1.z() = 0.0e0;
                                temp_p1.x() = -1.0e0 * (pe.b_ * temp_p1.y() + pe.c_ * temp_p1.z() + pe.d_) / pe.a_;
                                temp_p2.y() = rand(mt);
                                temp_p1.z() = rand(mt);
                                temp_p2.x() = -1.0e0 * (pe.b_ * temp_p2.y() + pe.c_ * temp_p2.z() + pe.d_) / pe.a_;
                                direction_vector = (temp_p2 - temp_p1).normalized();
                            }
                            else
                            {
                                return false;
                            }
                        }
                        else
                        {
                            return false;
                        }
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
                        if (HasIntersectionOfLine3dSegmentAndLine3dSegment(query, ray_point, edge_p1, edge_p2, intersection))
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
                            auto v_q1 = query - edge_p1;
                            auto v_21 = edge_p2 - edge_p1;
                            auto cross = (v_q1.normalized()).cross(v_21.normalized());
                            if (cross.dot(ref_cross) > -eps)
                            {
                                num_of_winding++;
                            }
                            else
                            {
                                num_of_winding--;
                            }
                        }
                    }
                    if (!try_other_direction)
                        break;
                }
            }
            if (num_of_winding == 0)
            {
                return false;
            } // if num_
            else
            {
                return true;
            }
        }
    }

    bool IsInsidePolygon3d(const Eigen::Vector3d &query, const std::vector<Eigen::Vector3d> &points, const double &eps)
    {
        return IsInsidePolygon3dRayCastingAlgorithm(query, points, eps);
        //return IsInsidePolygon3dWindingNumberAlgorithm(query, points, eps);
    }

    bool HasIntersectionLine3dAndPlane(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                       const PlaneEquation &plane_eq,
                                       Eigen::Vector3d &intersection,
                                       const double &eps)
    {
        if (IsOnPlane(p1, plane_eq))
        {
            intersection = p1;
            return true;
        } // if
        else if (IsOnPlane(p2, plane_eq))
        {
            intersection = p2;
            return true;
        } // else if
        {
            auto tol = std::max({std::abs(p2.x() - p1.x()), std::abs(p2.y() - p1.y()), std::abs(p2.z() - p1.z())}) * eps;
            if ((p2 - p1).norm() < tol)
            {
                // need to implement future
                // find distance and that return point
                return false;
            }
        }
        Eigen::Vector3d point_on_plane;
        if (std::abs(plane_eq.c_) > eps)
        {
            point_on_plane.x() = p2.x();
            point_on_plane.y() = p2.y();
            point_on_plane.z() = -1.0e0 * (plane_eq.a_ * point_on_plane.x() + plane_eq.b_ * point_on_plane.y() + plane_eq.d_) / plane_eq.c_;
        }
        else if (std::abs(plane_eq.b_) > eps)
        {
            point_on_plane.x() = p2.x();
            point_on_plane.z() = p2.z();
            point_on_plane.y() = -1.0e0 * (plane_eq.a_ * point_on_plane.x() + plane_eq.c_ * point_on_plane.z() + plane_eq.d_) / plane_eq.b_;
        }
        else if (std::abs(plane_eq.a_) > eps)
        {
            point_on_plane.y() = p2.y();
            point_on_plane.z() = p2.z();
            point_on_plane.x() = -1.0e0 * (plane_eq.b_ * point_on_plane.y() + plane_eq.c_ * point_on_plane.z() + plane_eq.d_) / plane_eq.a_;
        }
        else
        {
            return false;
        }
        Eigen::Vector3d normal_vec(plane_eq.a_, plane_eq.b_, plane_eq.c_);
        auto tol = (p2 - p1).norm() * eps;
        auto dn21 = normal_vec.dot(p2 - p1);
        if (std::abs(dn21) < tol)
        {
            return false;
        } // if dn21 <eps
        auto tmp = normal_vec.dot(point_on_plane - p1) / dn21;
        intersection = p1 + (p2 - p1) * tmp;
        return IsOnPlane(intersection, plane_eq);
    }
} // namespace geometry