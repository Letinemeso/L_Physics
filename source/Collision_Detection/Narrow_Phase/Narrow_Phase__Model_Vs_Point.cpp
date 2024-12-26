#include <Collision_Detection/Narrow_Phase/Narrow_Phase__Model_Vs_Point.h>

using namespace LPhys;


Narrow_Phase__Model_Vs_Point::Colliding_Modules Narrow_Phase__Model_Vs_Point::M_cast_modules(const Broad_Phase_Interface::Colliding_Pair& _maybe_modules) const
{
    Physics_Module_2D* model = LV::cast_variable<Physics_Module_2D>(_maybe_modules.first);

    if(model)
        return { model, LV::cast_variable<Physics_Module__Point>(_maybe_modules.second) };

    return { LV::cast_variable<Physics_Module_2D>(_maybe_modules.second), LV::cast_variable<Physics_Module__Point>(_maybe_modules.first) };
}

Intersection_Data Narrow_Phase__Model_Vs_Point::M_check_model_vs_point_intersection(const Colliding_Modules& _modules)
{
    L_ASSERT(_modules);

    const Polygon_Holder_Base* polygons = _modules.model->get_physical_model()->get_polygons();
    unsigned int polygons_amount = _modules.model->get_physical_model()->get_polygons_count();

    Intersection_Data result;

    for(unsigned int i=0; i<polygons_amount && !result; ++i)
    {
        if(!point_is_inside_polygon(_modules.point->point(), *polygons->get_polygon(i)))
            continue;

        result.point = _modules.point->point();
        result.first = _modules.model;
        result.second = _modules.point;
        result.first_collided_polygon_index = i;

        result.type = Intersection_Data::Type::intersection;
    }

    return result;
}



void Narrow_Phase__Model_Vs_Point::update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions)
{
    m_collisions.clear();

    for(Broad_Phase_Interface::Colliding_Pair_List::Const_Iterator it = _possible_collisions.begin(); !it.end_reached(); ++it)
    {
        Colliding_Modules colliding_modules = M_cast_modules(*it);
        if(!colliding_modules)
            continue;

        Intersection_Data id = M_check_model_vs_point_intersection(colliding_modules);
        if(!id)
            continue;

        m_collisions.push_back(id);
    }
}
