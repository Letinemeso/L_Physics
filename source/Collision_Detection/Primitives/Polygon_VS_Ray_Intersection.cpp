#include <Collision_Detection/Primitives/Polygon_VS_Ray_Intersection.h>

#include <Collision_Detection/Primitives/Polygon_Vs_Point_Intersection.h>

using namespace LPhys;


Polygon_VS_Ray_Intersection_Data LPhys::ray_intersects_polygon(const glm::vec3& _start, const glm::vec3& _direction, const Polygon& _polygon, float _tolerance)       //  note to self: this code was mostly written by deepseek, don't try to remember what it does
{
    Polygon_VS_Ray_Intersection_Data result;

    glm::vec3 edge1 = _polygon[1] - _polygon[0];
    glm::vec3 edge2 = _polygon[2] - _polygon[0];

    glm::vec3 pvec = LEti::Math::cross_product(_direction, edge2);
    float det = LEti::Math::dot_product(edge1, pvec);

    if (det < _tolerance && det > -_tolerance)
        return result;

    float inv_det = 1.0f / det;

    glm::vec3 tvec = _start - _polygon[0];
    float u = LEti::Math::dot_product(tvec, pvec) * inv_det;
    if (u < 0.0f || u > 1.0f)
        return result;

    glm::vec3 qvec = LEti::Math::cross_product(tvec, edge1);
    float v = LEti::Math::dot_product(_direction, qvec) * inv_det;
    if (v < 0.0f || u + v > 1.0f)
        return result;

    float t = dot(edge2, qvec) * inv_det;

    if(t < 0.0f)
        return result;

    result.intersection = true;
    result.direction_ratio = t;
    result.point = _start + _direction * t;
    return result;
}

Polygon_VS_Ray_Intersection_Data LPhys::segment_intersects_polygon(const glm::vec3& _start, const glm::vec3& _end, const Polygon& _polygon, float _tolerance)
{
    Polygon_VS_Ray_Intersection_Data result;

    glm::vec3 edge1 = _polygon[1] - _polygon[0];
    glm::vec3 edge2 = _polygon[2] - _polygon[0];

    glm::vec3 direction = _end - _start;

    glm::vec3 pvec = LEti::Math::cross_product(direction, edge2);
    float det = LEti::Math::dot_product(edge1, pvec);

    if (det < _tolerance && det > -_tolerance)
        return result;

    float inv_det = 1.0f / det;

    glm::vec3 tvec = _start - _polygon[0];
    float u = LEti::Math::dot_product(tvec, pvec) * inv_det;
    if (u < 0.0f || u > 1.0f)
        return result;

    glm::vec3 qvec = LEti::Math::cross_product(tvec, edge1);
    float v = LEti::Math::dot_product(direction, qvec) * inv_det;
    if (v < 0.0f || u + v > 1.0f)
        return result;

    float t = dot(edge2, qvec) * inv_det;

    if(t < 0.0f || t > 1.0f)
        return result;

    result.intersection = true;
    result.direction_ratio = t;
    result.point = _start + direction * t;
    return result;
}
