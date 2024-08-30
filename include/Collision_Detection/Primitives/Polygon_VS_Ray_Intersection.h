#pragma once

#include <Physical_Models/Polygon.h>

namespace LPhys
{

    struct Polygon_VS_Ray_Intersection_Data
    {
        bool intersection = false;
        glm::vec3 point;

        inline operator bool() const { return intersection; }
    };


    Polygon_VS_Ray_Intersection_Data ray_intersects_polygon(const glm::vec3& _start, const glm::vec3& _direction, const Polygon& _polygon);

}
