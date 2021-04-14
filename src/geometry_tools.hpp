#ifndef GEOMETRY_TOOLS_HPP_
#define GEOMETRY_TOOLS_HPP_

#include <Eigen/Dense>

namespace geometry
{
    bool IsParallel(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, const Eigen::Vector3d &p4, const double &eps = 1.0e-8);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeShortestLineBetweenSkewLine3dAndLine3d(
        const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
        const double &eps = 1.0e-8);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeShortestLineBetweenParallelLine3dAndLine3d(
        const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
        const double &eps = 1.0e-8);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeShortestLine3dBetweenLine3dAndLine3d(
        const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
        const double &eps = 1.0e-8);

    double ComputeShortestLengthLine3dAndLine3d(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                                const Eigen::Vector3d &p3, const Eigen::Vector3d &p4);

    double ComputeLengthPoint3dAndLine3d(const Eigen::Vector3d &query, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                         const double &eps = 1.0e-8);

    bool hasIntersectionOfLine3dAndLine3d(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                          const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                                          Eigen::Vector3d &intersection,
                                          const double &eps = 1.0e-8);

    bool IsOnLineSegment(const Eigen::Vector3d &query, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                         const double &eps = 1.0e-8);

    bool hasIntersectionOfLine3dSegmentAndLine3dSegment(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2,
                                                        const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                                                        Eigen::Vector3d &intersection,
                                                        const double &eps = 1.0e-8);

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

    PlaneEquation GetPlaneEquation(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3);

    bool IsOnPlane(const Eigen::Vector3d &query, const PlaneEquation &plane_equation,
                   const double &eps = 1.0e-8);

    bool IsInsidePolygon3dRayCastingAlgorithm(const Eigen::Vector3d &query, const std::vector<Eigen::Vector3d> &points,
                                              const double &eps = 1.0e-8);

    bool IsInsidePolygon3dWindingNumberAlgorithm(const Eigen::Vector3d &query, const std::vector<Eigen::Vector3d> &points,
                                                 const double &eps = 1.0e-8);

    bool IsInsidePolygon3d(const Eigen::Vector3d &query, const std::vector<Eigen::Vector3d> &points,
                           const double &eps = 1.0e-8);
} // namespace geometry


#endif // GEOMETRY_TOOLS_HPP_