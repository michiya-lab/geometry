#include <parent.hpp>

class child : public parent
{
public:
    child() { ; }
    virtual ~child() { ; }
    virtual void show_message() override;
}; // class child