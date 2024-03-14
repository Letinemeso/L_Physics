#pragma once

#include <Math_Stuff.h>

#include <Physical_Models/Polygon.h>
#include <Physical_Models/Border.h>
#include <Collision_Detection/Primitives/Segments_Intersection.h>


namespace LPhys
{

    bool point_is_inside_polygon(const glm::vec3& _point, const Polygon& _polygon);

}
