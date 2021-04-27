#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <set>
#include <random>
#include <vector>
#include <fstream>

#include <Eigen/Dense>

#include <geometry_tools.hpp>
#include <point3d.hpp>

namespace geometry
{
    double ComputeDistancePoint3dAndPlane3d(const Eigen::Vector3d &point, const PlaneEquation &plane_eq,
                                            const double eps = 1.0e-8);
    double ComputeDistancePoint3dAndPlane3d(const Eigen::Vector3d &point, const PlaneEquation &plane_eq,
                                            const double eps)
    {
        // https://keisan.casio.jp/exec/system/1202458240
        double denom_square = plane_eq.a_ * plane_eq.a_ + plane_eq.b_ * plane_eq.b_ + plane_eq.c_ * plane_eq.c_;
        {
            auto tol = std::max({std::abs(plane_eq.a_), std::abs(plane_eq.b_), std::abs(plane_eq.c_)}) * eps;
            if (std::abs(denom_square) < tol * tol)
                return 0.0e0;
        }
        double numer = plane_eq.a_ * point.x() + plane_eq.b_ * point.y() + plane_eq.c_ * point.z() + plane_eq.d_;
        {
            auto tol = std::max({std::abs(plane_eq.a_), std::abs(plane_eq.b_), std::abs(plane_eq.c_)}) *
                       std::max({std::abs(point.x()), std::abs(point.y()), std::abs(point.z())}) * eps;
            if (std::abs(numer) < tol)
                return 0.0e0;
        }
        return std::abs(numer) / std::sqrt(denom_square);
    }
    Eigen::Vector3d FindFootOfPerpendicularPoint3dOnPlane3d(const Eigen::Vector3d &point, const PlaneEquation &plane_eq,
                                                            const double eps = 1.0e-8);
    Eigen::Vector3d FindFootOfPerpendicularPoint3dOnPlane3d(const Eigen::Vector3d &point, const PlaneEquation &plane_eq,
                                                            const double eps)
    {
        // https://www.geeksforgeeks.org/find-the-foot-of-perpendicular-of-a-point-in-a-3-d-plane/
        // https://nekojara.city/math-point-project-on-plane
        // https://hiraocafe.com/note/plane_equation.html
        double denom = plane_eq.a_ * plane_eq.a_ + plane_eq.b_ * plane_eq.b_ + plane_eq.c_ * plane_eq.c_;
        {
            auto tol = std::max({std::abs(plane_eq.a_), std::abs(plane_eq.b_), std::abs(plane_eq.c_)}) * eps;
            if (denom < tol * tol)
                return point;
        }
        double numer = -1.0e0 * (plane_eq.a_ * point.x() + plane_eq.b_ * point.y() + plane_eq.c_ * point.z() + plane_eq.d_);
        auto normal_vec = Eigen::Vector3d(plane_eq.a_, plane_eq.b_, plane_eq.c_);
        return point + numer / denom * normal_vec;
    }
    //bool IsInPolyhedron(const Eigen::Vector3d &query, const std::vector<std::vector<Eigen::Vector3d>> &polyhedron_faces, const double &eps = 1.0e-8)
    //{
    //    for (const auto& face_points: polyhedron_faces)
    //    {
    //        if (face_points.size() < 3)
    //            return false;
    //        auto pe = GetPlaneEquation(face_points[0], face_points[1], face_points[2]);
    //        for (std::size_t i = 3; i < face_points.size(); ++i)
    //        {
    //
    //        } // for i
    //    } // for polyhedron_faces
    //}

    class Line3d
    {
    public:
        Line3d(Point3d *p1, Point3d *p2)
        {
            this->point_.first = p1;
            this->point_.second = p2;
        }
        virtual ~Line3d() { ; }
        long id_ = 0;
        std::pair<Point3d *, Point3d *> point_;

    public:                                          // oparator
        Line3d(const Line3d &) = default;            // copy constractor
        Line3d(Line3d &&) = default;                 // move constractor
        Line3d &operator=(const Line3d &) = default; // copy assignment operator
        Line3d &operator=(Line3d &&) = default;      // move assignment operator
    };                                               // class Line3d

    class Face3d
    {
    public:
        Face3d() { ; }
        virtual ~Face3d() { ; }
        long id_ = 0;
        std::vector<Point3d *> points_;

    public:                                          // oparator
        Face3d(const Face3d &) = default;            // copy constractor
        Face3d(Face3d &&) = default;                 // move constractor
        Face3d &operator=(const Face3d &) = default; // copy assignment operator
        Face3d &operator=(Face3d &&) = default;      // move assignment operator
    };                                               // class Face3d
} // namespace geometry

namespace
{
    void write_vtk(const std::string &fname, const std::vector<geometry::Face3d> &faces)
    {
        long num_of_points = 0;
        for (const auto &face : faces)
        {
            num_of_points += face.points_.size();
        }
        std::ofstream ofs(fname);
        ofs << "# vtk DataFile Version 2.0" << std::endl
            << "polygon" << std::endl
            << "ASCII" << std::endl
            << "DATASET UNSTRUCTURED_GRID" << std::endl
            << "POINTS " << num_of_points << " double" << std::endl;
        for (const auto &face : faces)
        {
            for (const auto &p : face.points_)
            {
                ofs << p->x_ << " "
                    << p->y_ << " "
                    << p->z_ << std::endl;
            } // for face
        }     // for faces
        ofs << "CELLS " << faces.size() << " " << num_of_points + faces.size() << " " << std::endl;
        {
            long count = 0;
            for (std::size_t iface = 0; iface < faces.size(); ++iface)
            {
                ofs << faces[iface].points_.size() << " ";
                for (std::size_t j = 0; j < faces[iface].points_.size(); ++j)
                {
                    ofs << count++ << " ";
                } // for j
            }     // for iface
        }
        ofs << std::endl;
        ofs << "CELL_TYPES " << faces.size() << std::endl;
        for (std::size_t i = 0; i < faces.size(); ++i)
        {
            ofs << "7 ";
        } // for i
        ofs << std::endl;
        ofs.close();
    }
} // namespace

int main()
{
    {
        auto p = Eigen::Vector3d(1.0e0, 2.0e0, 3.0e0);
        geometry::PlaneEquation pe;
        pe.a_ = 1.0e-20;
        pe.b_ = 4.0e0;
        pe.c_ = 3.0e0;
        pe.d_ = 5.0e0;
        std::cout << geometry::ComputeDistancePoint3dAndPlane3d(p, pe) << std::endl;
    }
        {
        auto p = Eigen::Vector3d(-1.0e0, 3.0e0, 4.0e0);
        geometry::PlaneEquation pe;
        pe.a_ = 1.0e0;
        pe.b_ = -2.0e0;
        pe.c_ = 0.0e0;
        pe.d_ = 0.0e0;
        auto intersect = geometry::FindFootOfPerpendicularPoint3dOnPlane3d(p, pe);
        std::cout << (p - intersect).norm() << std::endl;
        std::cout << geometry::ComputeDistancePoint3dAndPlane3d(p, pe) << std::endl;
    }
    return 0;
    constexpr double val_range = 1.0e+20;
    std::vector<Eigen::Vector3d> points;
    { // create triangle
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<double> rand(-val_range, val_range);
        points.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
        points.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
        points.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
        { // triangle vtk
            std::ofstream ofs("triangle.vtk");
            ofs << "# vtk DataFile Version 2.0" << std::endl
                << "polygon" << std::endl
                << "ASCII" << std::endl
                << "DATASET UNSTRUCTURED_GRID" << std::endl
                << "POINTS " << points.size() << " double" << std::endl;
            for (const auto &p : points)
            {
                ofs << std::setprecision(13)
                    << p.x() << " "
                    << p.y() << " "
                    << p.z() << std::endl;
            } // for points
            ofs << "CELLS 1 " << points.size() + 1 << " " << std::endl
                << points.size() << " ";
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                ofs << i << " ";
            } // for i
            ofs << std::endl
                << "CELL_TYPES 1" << std::endl
                << "7" << std::endl;
            ofs.close();
        }
    }
    { // intersection
        auto pe = geometry::GetPlaneEquation(points[0], points[1], points[2]);
        std::cout << "plane equation is "
                  << pe.a_ << " , "
                  << pe.b_ << " , "
                  << pe.c_ << " , "
                  << pe.d_ << " , "
                  << std::endl;
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<double> rand(-val_range, val_range);
        std::vector<Eigen::Vector3d> intersections_inside;
        std::vector<Eigen::Vector3d> intersections_outside;
        for (std::size_t i = 0; i < 100000; i++)
        {
            std::vector<Eigen::Vector3d> points_line;
            Eigen::Vector3d intersect;
            auto p1 = Eigen::Vector3d(rand(mt), rand(mt), rand(mt));
            auto p2 = Eigen::Vector3d(rand(mt), rand(mt), rand(mt));
            if (geometry::HasIntersectionLine3dAndPlane(p1, p2, pe, intersect))
            {
                if (geometry::IsInsidePolygon3d(intersect, points))
                {
                    intersections_inside.push_back(intersect);
                }
                else
                {
                    // std::cout << "check_outside" << std::endl;
                    // std::cout << "p1" << std::endl;
                    // std::cout << p1 << std::endl;
                    // std::cout << "p2" << std::endl;
                    // std::cout << p2 << std::endl;
                    // std::cout << "intersection" << std::endl;
                    // std::cout << intersect << std::endl;
                    intersections_outside.push_back(intersect);
                }
            }
        }
        std::cout << "inside/outside = "
                  << intersections_inside.size() << "/" << intersections_outside.size()
                  << std::endl;
        std::cout << "total = " << intersections_inside.size() + intersections_outside.size() << std::endl;
        { // intersection inside
            std::ofstream ofs("inside.vtk");
            ofs << "# vtk DataFile Version 2.0" << std::endl
                << "polygon" << std::endl
                << "ASCII" << std::endl
                << "DATASET UNSTRUCTURED_GRID" << std::endl
                << "POINTS " << intersections_inside.size() << " double" << std::endl;
            for (const auto &p : intersections_inside)
            {
                ofs << std::setprecision(13)
                    << p.x() << " "
                    << p.y() << " "
                    << p.z() << std::endl;
            } // for points
            ofs << "CELLS " << intersections_inside.size() << " " << intersections_inside.size() * 2 << " " << std::endl;
            for (std::size_t i = 0; i < intersections_inside.size(); ++i)
            {
                ofs << " 1 " << i << " ";
            } // for i
            ofs << std::endl;
            ofs << "CELL_TYPES " << intersections_inside.size() << std::endl;
            for (std::size_t i = 0; i < intersections_inside.size(); ++i)
            {
                ofs << " 1 ";
            } // for i
            ofs << std::endl;
            ofs.close();
        }
        { // intersection outside
            std::ofstream ofs("outside.vtk");
            ofs << "# vtk DataFile Version 2.0" << std::endl
                << "point" << std::endl
                << "ASCII" << std::endl
                << "DATASET UNSTRUCTURED_GRID" << std::endl
                << "POINTS " << intersections_outside.size() << " double" << std::endl;
            for (const auto &p : intersections_outside)
            {
                ofs << std::setprecision(13)
                    << p.x() << " "
                    << p.y() << " "
                    << p.z() << std::endl;
            } // for points
            ofs << "CELLS " << intersections_outside.size() << " " << intersections_outside.size() * 2 << " " << std::endl;
            for (std::size_t i = 0; i < intersections_outside.size(); ++i)
            {
                ofs << " 1 " << i << " ";
            } // for i
            ofs << std::endl;
            ofs << "CELL_TYPES " << intersections_outside.size() << std::endl;
            for (std::size_t i = 0; i < intersections_outside.size(); ++i)
            {
                ofs << " 1 ";
            } // for i
            ofs << std::endl;
            ofs.close();
        }
    }
    return 0;
}