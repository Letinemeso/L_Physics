#pragma once

#include <vec3.hpp>

#include <Modules/Physics_Module.h>

namespace LPhys
{

    class Polygon;

    struct Intersection_Data
    {
    public:
        enum class Type
        {
            none = 0,
            intersection,
            same_line
        };

    public:
        Type type = Type::none;
        glm::vec3 point{0.0f, 0.0f, 0.0f};
        glm::vec3 normal{0.0f, 0.0f, 0.0f};
        float depth = 0.0f;
        Physics_Module* first = nullptr;
        Physics_Module* second = nullptr;
        unsigned int first_collided_polygon_index = 0;
        unsigned int second_collided_polygon_index = 0;
        float time_of_intersection_ratio = 1.0f;

    public:
        Intersection_Data();
        Intersection_Data(Type _type);
        Intersection_Data(Type _type, const glm::vec3& _point);
        Intersection_Data(const Intersection_Data& _other);
        Intersection_Data(Intersection_Data&& _other);
        void operator=(const Intersection_Data& _other);

    public:
        inline operator bool() { return type != Type::none; }
    };

}
