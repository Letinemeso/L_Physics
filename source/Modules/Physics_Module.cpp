#include <Modules/Physics_Module.h>

using namespace LPhys;


bool Physics_Module::may_intersect_with_other(const Physics_Module& _other) const
{
    return m_border && _other.m_border;
}

bool Physics_Module::intersects_with_border(const Border& _border) const
{
    return m_border && _border;
}
