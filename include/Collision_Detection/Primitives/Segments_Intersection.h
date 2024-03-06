#pragma once

#include <Collision_Detection/Primitives/Lines_Intersection.h>


namespace LPhys
{

    Lines_Intersection_Data calculate_segments_intersection(const glm::vec3& _first_start, const glm::vec3& _first_end, const glm::vec3& _second_start, const glm::vec3& _second_end);

}
