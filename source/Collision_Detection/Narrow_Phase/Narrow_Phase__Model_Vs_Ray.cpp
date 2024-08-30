#include <Collision_Detection/Narrow_Phase/Narrow_Phase__Model_Vs_Ray.h>

#include <Collision_Detection/Primitives/Polygon_VS_Ray_Intersection.h>

using namespace LPhys;


Narrow_Phase__Model_Vs_Ray::Colliding_Modules Narrow_Phase__Model_Vs_Ray::M_cast_modules(const Broad_Phase_Interface::Colliding_Pair& _maybe_modules) const
{
    const Physics_Module_2D* model = LV::cast_variable<Physics_Module_2D>(_maybe_modules.first);

    if(model)
        return { model, LV::cast_variable<Physics_Module__Ray>(_maybe_modules.second) };

    return { LV::cast_variable<Physics_Module_2D>(_maybe_modules.second), LV::cast_variable<Physics_Module__Ray>(_maybe_modules.first) };
}

Intersection_Data Narrow_Phase__Model_Vs_Ray::M_check_model_vs_ray_intersection(const Colliding_Modules& _modules)
{
    L_ASSERT(_modules);

    const Polygon_Holder_Base* polygons = _modules.model->get_physical_model()->get_polygons();
    unsigned int polygons_amount = _modules.model->get_physical_model()->get_polygons_count();

    Intersection_Data result;

    for(unsigned int i=0; i<polygons_amount && !result; ++i)
    {
        Polygon_VS_Ray_Intersection_Data id = ray_intersects_polygon(_modules.ray->current_start(), _modules.ray->current_direction(), *polygons->get_polygon(i));

        if(!id)
            continue;

        result.point = id.point;
        result.first = _modules.model;
        result.second = _modules.ray;
        result.first_collided_polygon_index = i;

        result.type = Intersection_Data::Type::intersection;
    }

    return result;
}



void Narrow_Phase__Model_Vs_Ray::update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions)
{
    m_collisions.clear();

    for(Broad_Phase_Interface::Colliding_Pair_List::Const_Iterator it = _possible_collisions.begin(); !it.end_reached(); ++it)
    {
        Colliding_Modules colliding_modules = M_cast_modules(*it);
        if(!colliding_modules)
            continue;

        Intersection_Data id = M_check_model_vs_ray_intersection(colliding_modules);
        if(!id)
            continue;

        m_collisions.push_back(id);
    }
}
