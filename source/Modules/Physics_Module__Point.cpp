#include <Modules/Physics_Module__Point.h>

using namespace LPhys;


void Physics_Module__Point::set_point(const glm::vec3& _point)
{
    m_raw_point = _point;

    if(transformation_data())
        m_current_point = transformation_data()->matrix() * glm::vec4(_point, 1.0f);
    else
        m_current_point = _point;
}



bool Physics_Module__Point::may_intersect_with_other(const Physics_Module& _other) const
{
    return true;
}

bool Physics_Module__Point::intersects_with_border(const Border& _border) const
{
    return _border.point_is_inside(m_current_point);
}



void Physics_Module__Point::update(float _dt)
{
    if(!transformation_data())
        return;

    m_current_point = transformation_data()->matrix() * glm::vec4(m_raw_point, 1.0f);
}





BUILDER_STUB_CONSTRUCTION_FUNC(Physics_Module_Stub__Point) BUILDER_STUB_CONSTRUCTION_FUNC_DEFAULT_IMPL

BUILDER_STUB_INITIALIZATION_FUNC(Physics_Module_Stub__Point)
{
    BUILDER_STUB_PARENT_INITIALIZATION;
    BUILDER_STUB_CAST_PRODUCT;

    product->set_point(point);
}
