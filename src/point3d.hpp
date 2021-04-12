#ifndef POINT3D_HPP_
#define POINT3D_HPP_


namespace geometry
{
    class Point3d
    {
    public:
        Point3d(const double &x, const double &y, const double &z)
        {
            this->x_ = x;
            this->y_ = y;
            this->z_ = z;
        }
        Point3d(const long &id, const double &x, const double &y, const double &z)
        {
            this->id_ = id;
            this->x_ = x;
            this->y_ = y;
            this->z_ = z;
        }
        virtual ~Point3d() { ; }
        long id_ = 0;
        double x_ = 0.0e0;
        double y_ = 0.0e0;
        double z_ = 0.0e0;

    private:
        /* code */
    public:                                            // oparator
        Point3d(const Point3d &) = default;            // copy constractor
        Point3d(Point3d &&) = default;                 // move constractor
        Point3d &operator=(const Point3d &) = default; // copy assignment operator
        Point3d &operator=(Point3d &&) = default;      // move assignment operator
    };                                                 // class Point3d

    class Line3d
    {
    public:
        Line3d(){ ; }
        virtual ~Line3d(){ ; }
        Point3d *p1 = nullptr;
        Point3d *p2 = nullptr;

    private:
        /* code */
    public: // access member
        /* code */
    public: // oparator
        Line3d(const Line3d&) = default; // copy constractor
        Line3d(Line3d&&) = default; // move constractor
        Line3d &operator=(const Line3d&) = default; // copy assignment operator
        Line3d &operator=(Line3d&&) = default; // move assignment operator
    }; // class Line3d
} // namespace geometry

#endif // POINT3D_HPP_