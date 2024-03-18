#include <Modules/Physics_Module.h>

using namespace LPhys;


void Physics_Module::expand_border(Border& _border) const
{

}

bool Physics_Module::may_intersect_with_other(const Physics_Module& _other) const
{
    return true;
}

bool Physics_Module::intersects_with_border(const Border& _border) const
{
    return true;
}
