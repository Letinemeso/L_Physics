#pragma once

#include <vec3.hpp>

#include <Data_Structures/Vector.h>

namespace LPhys
{

    class Physics_Module;

    struct Intersection_Data
    {
    public:
        bool intersection = false;
        LDS::Vector<glm::vec3> points;
        LDS::Vector<float> depths_per_point;
        glm::vec3 normal{0.0f, 0.0f, 0.0f};
        float depth = 0.0f;
        Physics_Module* first = nullptr;
        Physics_Module* second = nullptr;
        unsigned int first_collided_polygon_index = 0;
        unsigned int second_collided_polygon_index = 0;
        float time_of_intersection_ratio = 1.0f;
        float time_of_intersection_ratio_step = 1.0f;

    public:
        Intersection_Data();
        Intersection_Data(const Intersection_Data& _other);
        Intersection_Data(Intersection_Data&& _other);
        void operator=(const Intersection_Data& _other);

    public:
        inline operator bool() { return intersection; }
    };

}
