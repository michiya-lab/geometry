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
        // https://www.geisya.or.jp/~mwm48961/linear_algebra/line_plane3.htm
        // ex. 1
        std::cout << "ex. 1" << std::endl;
        geometry::PlaneEquation pe;
        pe.a_ = 1.0e0;
        pe.b_ = -2.0e0;
        pe.c_ = 3.0e0;
        pe.d_ = 10.0e0;
        Eigen::Vector3d intersection;
        std::cout << HasIntersectionLine3dAndPlane(Eigen::Vector3d(1.0e0, 1.0e0, -4.0e0),
                                                   Eigen::Vector3d(-5.0e0, 3.0e0, 0.0e0),
                                                   pe, intersection)
                  << std::endl;
        std::cout << intersection << std::endl;
    }
    {
        // https://www.geisya.or.jp/~mwm48961/linear_algebra/line_plane3.htma
        // ex. 1.1
        std::cout << "ex. 1.1" << std::endl;
        geometry::PlaneEquation pe;
        pe.a_ = 2.0e0;
        pe.b_ = -1.0e0;
        pe.c_ = 2.0e0;
        pe.d_ = -2.0e0;
        Eigen::Vector3d intersection;
        std::cout << HasIntersectionLine3dAndPlane(Eigen::Vector3d(3.0e0, -4.0e0, 5.0e0),
                                                   Eigen::Vector3d(5.0e0, -1.0e0, 9.0e0),
                                                   pe, intersection)
                  << std::endl;
        std::cout << intersection << std::endl;
    }
    {
        // https://www.geisya.or.jp/~mwm48961/linear_algebra/line_plane3.htma
        // ex. 1.2
        std::cout << "ex. 1.2" << std::endl;
        geometry::PlaneEquation pe;
        pe.a_ = 1.0e0;
        pe.b_ = 1.0e0;
        pe.c_ = 2.0e0;
        pe.d_ = 1.0e0;
        Eigen::Vector3d intersection;
        std::cout << HasIntersectionLine3dAndPlane(Eigen::Vector3d(2.0e0, 0.0e0, 1.0e0),
                                                   Eigen::Vector3d(3.0e0, -2.0e0, 4.0e0),
                                                   pe, intersection)
                  << std::endl;
        std::cout << intersection << std::endl;
    }
    {
        // https://www.geisya.or.jp/~mwm48961/linear_algebra/line_plane3.htma
        // ex. 1.3
        std::cout << "ex. 1.3" << std::endl;
        geometry::PlaneEquation pe;
        pe.a_ = 3.0e0;
        pe.b_ = -2.0e0;
        pe.c_ = 4.0e0;
        pe.d_ = -9.0e0;
        Eigen::Vector3d intersection;
        std::cout << HasIntersectionLine3dAndPlane(Eigen::Vector3d(-1.0e0, 2.0e0, 1.0e0),
                                                   Eigen::Vector3d(-3.0e0, 5.0e0, 1.0e0),
                                                   pe, intersection)
                  << std::endl;
        std::cout << intersection << std::endl;
    }
    return 0;
    {
        std::vector<Eigen::Vector3d> points;
        { // create triangl
            std::random_device rnd;
            std::mt19937 mt(rnd());
            std::uniform_real_distribution<double> rand(-1.0e+0, 1.0e+0);
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
        }std::vector<Eigen::Vector3d> points_line;
        { // create triangl
            std::random_device rnd;
            std::mt19937 mt(rnd());
            std::uniform_real_distribution<double> rand(-1.0e+0, 1.0e+0);
            points_line.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
            points_line.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
            { // triangle vtk
                std::ofstream ofs("line.vtk");
                ofs << "# vtk DataFile Version 2.0" << std::endl
                    << "polygon" << std::endl
                    << "ASCII" << std::endl
                    << "DATASET UNSTRUCTURED_GRID" << std::endl
                    << "POINTS " << points_line.size() << " double" << std::endl;
                for (const auto &p : points_line)
                {
                    ofs << std::setprecision(13)
                        << p.x() << " "
                        << p.y() << " "
                        << p.z() << std::endl;
                } // for points
                ofs << "CELLS 1 " << points_line.size() + 1 << " " << std::endl
                    << points_line.size() << " ";
                for (std::size_t i = 0; i < points_line.size(); ++i)
                {
                    ofs << i << " ";
                } // for i
                ofs << std::endl
                    << "CELL_TYPES 1" << std::endl
                    << "3" << std::endl;
                ofs.close();
            }
        }
        {
            auto pe = geometry::GetPlaneEquation(points[0], points[1], points[2]);
            Eigen::Vector3d intersection;
            std::cout << HasIntersectionLine3dAndPlane(points_line[0], points_line[1], pe, intersection) << std::endl;
            std::cout << "point1" << std::endl;
            std::cout << points[0] << std::endl;
            std::cout << "point2" << std::endl;
            std::cout << points[1] << std::endl;
            std::cout << "point3" << std::endl;
            std::cout << points[2] << std::endl;
            std::cout << "point_line1" << std::endl;
            std::cout << points_line[0] << std::endl;
            std::cout << "point_line2" << std::endl;
            std::cout << points_line[1] << std::endl;
            std::cout << "intersection" << std::endl;
            std::cout << intersection << std::endl;
        }
    }
    return 0;
    {
        std::vector<std::unique_ptr<geometry::Point3d>> points;
        std::vector<std::unique_ptr<geometry::Face3d>> faces;
        {
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 6.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 6.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 4.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 4.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 6.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 4.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 4.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 4.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 6.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 6.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 4.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 6.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 4.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 4.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 4.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 4.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 4.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 4.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 6.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 6.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 4.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 4.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 6.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 4.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 6.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 6.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 6.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 6.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 6.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-5.50E+01, 6.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 4.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 4.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 6.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 6.00E+01, 2.00E+01));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 4.00E+01, 0.00E+00));
            points.push_back(std::make_unique<geometry::Point3d>(-3.50E+01, 6.00E+01, 0.00E+00));
        }
        {
            for (std::size_t i = 0; i < points.size(); i += 3)
            {
                auto face = std::make_unique<geometry::Face3d>();
                face->points_.push_back(points[i].get());
                face->points_.push_back(points[i + 1].get());
                face->points_.push_back(points[i + 2].get());
                faces.push_back(std::move(face));
            } // for i
        }
        return 0;
        {
            std::vector<geometry::Face3d> faces_print;
            for (const auto& f: faces)
            {
                faces_print.push_back(*f.get());
            } // for face
            write_vtk("faces.vtk", faces_print);
        }
    }
    return 0;
    {
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<double> rand(-1.0e+16, 1.0e+16);
        std::vector<Eigen::Vector3d> points;
        { // create polygon
            points.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
            points.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
            points.push_back(Eigen::Vector3d(rand(mt), rand(mt), rand(mt)));
            auto pe = geometry::GetPlaneEquation(points[0], points[1], points[2]);
            for (std::size_t i = 0; i < 2; ++i)
            {
                if (std::abs(pe.c_) > 1.0e-8)
                {
                    Eigen::Vector3d p1;
                    p1.x() = rand(mt);
                    p1.y() = rand(mt);
                    p1.z() = -1.0e0 * (pe.a_ * p1.x() + pe.b_ * p1.y() + pe.d_) / pe.c_;
                    points.push_back(p1);
                }
            } // for i
            { // triangle vtk
                std::ofstream ofs("polygon.vtk");
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
//
        auto pe = geometry::GetPlaneEquation(points[0], points[1], points[2]);
        std::vector<Eigen::Vector3d> queries;
        std::size_t count = 0;
        for (std::size_t i = 0; i < 10000 * 10; ++i)
        {
            Eigen::Vector3d p;
            p.x() = rand(mt);
            p.y() = rand(mt);
            p.z() = -1.0e0 * (pe.a_ * p.x() + pe.b_ * p.y() + pe.d_) / pe.c_;
            queries.push_back(p);
            if (geometry::IsInsidePolygon3d(p, points))
            {
                count++;
            }
        } // for i
        std::cout << "inside / outside is " << count << " / " << queries.size() - count << std::endl;
        {
            std::ofstream ofs;
            ofs.open("points_inside.vtk");
            ofs << "# vtk DataFile Version 2.0" << std::endl
                << "points" << std::endl
                << "ASCII" << std::endl
                << "DATASET UNSTRUCTURED_GRID " << std::endl
                << "POINTS " << count << " double"
                << std::endl;
            for (std::size_t i = 0; i < queries.size(); ++i)
            {
                if (geometry::IsInsidePolygon3d(queries[i], points))
                {
                    ofs << std::setprecision(13)
                        << queries[i].x() << "  "
                        << queries[i].y() << "  "
                        << queries[i].z() << std::endl;
                }
            } // for i
            ofs << "CELLS " << count << " " << count * 2 << std::endl;
            for (std::size_t i = 0; i < count; ++i)
            {
                ofs << "1 " << i << " ";
            } // for i
            ofs << std::endl;
            ofs << "CELL_TYPES " << count << std::endl;
            for (std::size_t i = 0; i < count; ++i)
            {
                ofs << "1 ";
            } // for i
            ofs << std::endl;
            ofs.close();
        }
        {
            std::ofstream ofs;
            ofs.open("points_outside.vtk");
            ofs << "# vtk DataFile Version 2.0" << std::endl
                << "points" << std::endl
                << "ASCII" << std::endl
                << "DATASET UNSTRUCTURED_GRID " << std::endl
                << "POINTS " << queries.size() - count << " double"
                << std::endl;
            for (std::size_t i = 0; i < queries.size(); ++i)
            {
                if (!geometry::IsInsidePolygon3d(queries[i], points))
                {
                    ofs << std::setprecision(13)
                        << queries[i].x() << "  "
                        << queries[i].y() << "  "
                        << queries[i].z() << std::endl;
                }
            } // for i
            ofs << "CELLS " << queries.size() - count << " " << (queries.size() - count) * 2 << std::endl;
            for (std::size_t i = 0; i < queries.size() - count; ++i)
            {
                ofs << "1 " << i << " ";
            } // for i
            ofs << std::endl;
            ofs << "CELL_TYPES " << queries.size() - count << std::endl;
            for (std::size_t i = 0; i < queries.size() - count; ++i)
            {
                ofs << "1 ";
            } // for i
            ofs << std::endl;
            ofs.close();
        }
    }
    return 0;
    }
