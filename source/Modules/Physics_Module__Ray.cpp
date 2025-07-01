#include <Modules/Physics_Module__Ray.h>

#include <algorithm>

using namespace LPhys;


void Physics_Module__Ray::expand_border(Border& _border) const
{
    _border.consider_point(m_current_start);
}

bool Physics_Module__Ray::may_intersect_with_other(const Physics_Module& _other) const
{
    return true;
}

bool Physics_Module__Ray::intersects_with_border(const Border& _border) const
{
    float tmin = (m_current_direction.x != 0) ? (_border.offset().x - m_current_start.x) / m_current_direction.x : -std::numeric_limits<float>::infinity();
    float tmax = (m_current_direction.x != 0) ? ((_border.offset().x + _border.size().x) - m_current_start.x) / m_current_direction.x : std::numeric_limits<float>::infinity();

    if (tmin > tmax)
        std::swap(tmin, tmax);

    float tymin = (m_current_direction.y != 0) ? (_border.offset().y - m_current_start.y) / m_current_direction.y : -std::numeric_limits<float>::infinity();
    float tymax = (m_current_direction.y != 0) ? ((_border.offset().y + _border.size().y) - m_current_start.y) / m_current_direction.y : std::numeric_limits<float>::infinity();

    if (tymin > tymax) std::swap(tymin, tymax);

    if ((tmin > tymax) || (tymin > tmax))
        return false;

    tmin = std::max(tmin, tymin);
    tmax = std::min(tmax, tymax);

    float tzmin = (m_current_direction.z != 0) ? (_border.offset().z - m_current_start.z) / m_current_direction.z : -std::numeric_limits<float>::infinity();
    float tzmax = (m_current_direction.z != 0) ? ((_border.offset().z + _border.size().z) - m_current_start.z) / m_current_direction.z : std::numeric_limits<float>::infinity();

    if (tzmin > tzmax)
        std::swap(tzmin, tzmax);

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;

    tmin = std::max(tmin, tzmin);
    tmax = std::min(tmax, tzmax);

    return (tmax >= 0);
}



void Physics_Module__Ray::update(float _dt)
{
    m_current_start = transformation_data()->matrix() * glm::vec4(m_base_start, 1.0f);
    m_current_direction = transformation_data()->rotation_matrix() * glm::vec4(m_base_direction, 1.0f);
}





BUILDER_STUB_DEFAULT_CONSTRUCTION_FUNC(Physics_Module_Stub__Ray)

BUILDER_STUB_INITIALIZATION_FUNC(Physics_Module_Stub__Ray)
{
    BUILDER_STUB_PARENT_INITIALIZATION;
    BUILDER_STUB_CAST_PRODUCT;

    product->set_start(start);
    product->set_direction(direction);
}
