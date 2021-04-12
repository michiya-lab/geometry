namespace geometry
{
    class database
    {
    public:
        database() { ; }
        virtual ~database() { ; }
        /* code */
    private:
    public: // access member
        /* code */
    public:                                              // oparator
        database(const database &) = default;            // copy constractor
        database(database &&) = default;                 // move constractor
        database &operator=(const database &) = default; // copy assignment operator
        database &operator=(database &&) = default;      // move assignment operator
    };                                                   // class database
} // namespace geometry