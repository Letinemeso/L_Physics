#include <Collision_Detection/Primitives/Polygon_VS_Ray_Intersection.h>

#include <Collision_Detection/Primitives/Polygon_Vs_Point_Intersection.h>

using namespace LPhys;


Polygon_VS_Ray_Intersection_Data LPhys::ray_intersects_polygon(const glm::vec3& _start, const glm::vec3& _direction, const Polygon& _polygon, float _tolerance)       //  note to self: this code was mostly written by chat-gpt, don't try to remember what it does
{
    Polygon_VS_Ray_Intersection_Data result;

    glm::vec3 edgeAB = _polygon[1] - _polygon[0];
    glm::vec3 edgeAC = _polygon[2] - _polygon[0];
    glm::vec3 directionCrossEdgeAC =  LEti::Math::cross_product(_direction, edgeAC);
    float determinant = LEti::Math::dot_product(edgeAB, directionCrossEdgeAC);

    if (LEti::Math::floats_are_equal(determinant, 0.0f))
        return result;

    float inverseDeterminant = 1.0f / determinant;
    glm::vec3 distanceFromAtoLineStart = _start - _polygon[0];
    float u = inverseDeterminant * LEti::Math::dot_product(distanceFromAtoLineStart, directionCrossEdgeAC);

    if (u < 0.0 || u > 1.0)
        return result;

    glm::vec3 distanceCrossEdgeAB = LEti::Math::cross_product(distanceFromAtoLineStart, edgeAB);
    float v = inverseDeterminant * LEti::Math::dot_product(_direction, distanceCrossEdgeAB);

    if (v < 0.0 || u + v > 1.0)
        return result;

    float t = inverseDeterminant * LEti::Math::dot_product(edgeAC, distanceCrossEdgeAB);

    if (t <= 1e-6)
        return result;

    glm::vec3 intersectionPoint = _start + _direction * t;
    result.intersection = point_is_inside_polygon(intersectionPoint, _polygon, _tolerance);
    if(!result)
        return result;

    glm::vec3 direction_to_intersection_point = intersectionPoint - _start;
    for(unsigned int i=0; i<3; ++i)
    {
        if(_direction[i] < 0.0f && direction_to_intersection_point[i] >= 0.0f)
            return result;
    }

    result.point = intersectionPoint;
    return result;
}
