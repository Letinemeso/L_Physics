#pragma once

#include <Data_Structures/Vector.h>

#include <Physical_Models/Physical_Model.h>
#include <Modules/Physics_Module__Mesh.h>


namespace LPhys
{

    struct Polygons_Pair
    {
        const Polygon* first = nullptr;
        const Polygon* second = nullptr;
    };

    using Possible_Colliding_Polygons = LDS::Vector<Polygons_Pair>;


    Possible_Colliding_Polygons find_possible_colliding_polygons(const Polygon_Holder_Base& _polygons_1,
                                                                 const Polygon_Holder_Base& _polygons_2,
                                                                 unsigned int _min_polygons_in_area);

    Possible_Colliding_Polygons find_possible_colliding_polygons(const Polygon_Holder_Base& _polygons_1,
                                                                 const LDS::Vector<Border>& _borders_cache_1,
                                                                 const Polygon_Holder_Base& _polygons_2,
                                                                 const LDS::Vector<Border>& _borders_cache_2,
                                                                 unsigned int _min_polygons_in_area,
                                                                 const Border& _initial_area);

}
