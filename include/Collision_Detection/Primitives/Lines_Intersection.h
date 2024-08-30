#pragma once

#include <vec3.hpp>

#include <Math_Stuff.h>


namespace LPhys
{

    struct Lines_Intersection_Data
    {
        enum Intersection_Type : unsigned int
        {
            None,
            Intersection,
            Same_Line
        };

        Intersection_Type intersection = None;
        glm::vec3 point;

        inline operator bool() const { return intersection != None; }
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
        Line();
        Line(const glm::vec3& _point_1, const glm::vec3& _point_2);

    public:
        void init(const glm::vec3& _point_1, const glm::vec3& _point_2);

    private:
        float M_calculate_multiplier_by_component(Components _component, float _value) const;
        unsigned int M_get_easy_solution_component(const Line& _with) const;
        bool M_matches_with(const Line& _with) const;

    public:
        bool contains_point(const glm::vec3& _point, float _tolerance = 0.0001f) const;
        bool can_be_solved_by_component(Components _component) const;
        glm::vec3 solve_by_component(Components _component, float _value) const;
        glm::vec3 solve_by_multiplier(float _multiplier) const;
        Lines_Intersection_Data calculate_intersection_with(const Line& _with, float _tolerance = 0.0001f) const;

    public:
        inline bool can_be_solved_by_component(unsigned int _component) const { return can_be_solved_by_component((Components)_component); }
        inline glm::vec3 solve_by_component(unsigned int _component, float _value) const { return solve_by_component((Components)_component, _value); }

    };

}
