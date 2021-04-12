////////////////////////////////////////////////////////////////////////
///  Copyright (c) 2015-2021, Michiya Imachi, All Rights Reserved.
////////////////////////////////////////////////////////////////////////

namespace geometry
{
    using int_type = long;
    using uint_type = unsigned long;
    using real_type = double;
} // namespace geometry

namespace geometry
{
    class geometry_interface
    {
    public:
        geometry_interface() { ; }
        virtual ~geometry_interface() { ; }
        geometry::int_type id() { return this->m_id; }

    private:
        geometry::int_type m_id = 0;

    public:                                                                  // oparator
        geometry_interface(const geometry_interface &) = default;            // copy constractor
        geometry_interface(geometry_interface &&) = default;                 // move constractor
        geometry_interface &operator=(const geometry_interface &) = default; // copy assignment operator
        geometry_interface &operator=(geometry_interface &&) = default;      // move assignment operator
    };                                                                       // class geometry_interface
} // namespace geometry
