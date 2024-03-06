#pragma once

#include <vec3.hpp>

#include <Math_Stuff.h>


namespace LPhys
{

    struct Lines_Intersection_Data
    {
        bool has_intersection = false;
        glm::vec3 point;
    };


    class Line
    {
    public:
        enum Components
        {
            x = 0,
            y = 1,
            z = 2,
            amount = 3
        };

    private:
        glm::vec3 m_direction;
        glm::vec3 m_initial_offset;

    public:
        Line(const glm::vec3& _point_1, const glm::vec3& _point_2);

    private:
        float M_calculate_multiplier_by_component(Components _component, float _value) const;

    public:
        bool contains_point(const glm::vec3& _point, float _tolerance = 0.0001f) const;
        bool can_be_solved_by_component(Components _component) const;
        glm::vec3 solve_by_component(Components _component, float _value) const;
        glm::vec3 solve_by_multiplier(float _multiplier) const;
        Lines_Intersection_Data calculate_intersection_with(const Line& _with) const;

    public:
        inline bool can_be_solved_by_component(unsigned int _component) const { return can_be_solved_by_component((Components)_component); }
        inline glm::vec3 solve_by_component(unsigned int _component, float _value) const { return solve_by_component((Components)_component, _value); }

    };

}
