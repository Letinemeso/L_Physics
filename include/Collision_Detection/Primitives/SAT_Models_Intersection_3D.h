#pragma once

#include <Collision_Detection/Primitives/Primitives_Intersection_Detector.h>


namespace LPhys
{

    class SAT_Models_Intersection_3D : public Primitives_Intersection_Detector
    {
    private:
        unsigned int m_min_polygons_for_optimization = 8;   //  if polygons amount on any mesh exceeds this value, optimization algotythm is used (something with space partitioning)
        float m_plane_contact_priority_ratio = 3.0f;           //  if edge contact has depth difference with plane contact less than this value, plane contact is prioritized

    public:
        inline void set_min_polygons_for_optimization(unsigned int _value) { m_min_polygons_for_optimization = _value; }
        inline void set_plane_contact_priority_ratio(float _value) { m_plane_contact_priority_ratio = _value; }

    private:
        struct Polygons_Intersection_Data
        {
            bool intersection = false;
            glm::vec3 point;
            glm::vec3 normal;
            float depth = 0.0f;
        };

        struct MinMax_Pair
        {
            float min = std::numeric_limits<float>::max();
            float max = -std::numeric_limits<float>::max();

            inline void add_value(float _value)
            {
                if(_value < min)
                    min = _value;
                if(_value > max)
                    max = _value;
            }

            inline bool value_is_between(float _value)
            {
                constexpr float tolerance = 1e-9f;
                return _value > min - tolerance && _value < max - tolerance;
            }
        };

        struct Axis_Intersection
        {
            bool intersection = false;
            float depth = 0.0f;
            bool flip_normal = false;
        };

        struct Common_Intersection_Data
        {
            LDS::Vector<glm::vec3> points = {100};
            glm::vec3 push_out_vector = { 0.0f, 0.0f, 0.0f };
            float total_depth = 0.0f;
            unsigned int intersections_amount = 0;
        };

        struct Basis
        {
            glm::vec3 axis_0, axis_1;

            Basis(const glm::vec3& _normal)
            {
                if (fabsf(_normal.x) > fabsf(_normal.z))
                    axis_0 = glm::vec3(-_normal.y, _normal.x, 0.0f);
                else
                    axis_0 = glm::vec3(0.0f, -_normal.z, _normal.y);

                axis_1 = glm::cross(_normal, axis_0);

                LST::Math::shrink_vector_to_1(axis_0);
                LST::Math::shrink_vector_to_1(axis_1);
            }
        };

    private:
        Polygons_Intersection_Data M_calculate_intersection_point(const Polygon& _first, const Polygon& _second) const;

        Axis_Intersection M_check_intersection_on_axis(const glm::vec3& _axis, const Polygon& _first, const Polygon& _second) const;
        Polygons_Intersection_Data M_check_triangles_intersection(const Polygon& _first, const Polygon& _second) const;

        Common_Intersection_Data M_calculate_common_intersection(const Polygon_Holder_Base* _polygon_holder_1, const LDS::Vector<Border>& _polygons_borders_cache_1,
                                                                 const Polygon_Holder_Base* _polygon_holder_2, const LDS::Vector<Border>& _polygons_borders_cache_2) const;
        Common_Intersection_Data M_calculate_common_intersection_optimized(const Polygon_Holder_Base* _polygon_holder_1, const Border& _border_1,
                                                                           const LDS::Vector<Border>& _polygons_borders_cache_1,
                                                                           const Polygon_Holder_Base* _polygon_holder_2, const Border& _border_2,
                                                                           const LDS::Vector<Border>& _polygons_borders_cache_2) const;

        LDS::Vector<glm::vec3> M_find_extreme_contacts(const LDS::Vector<glm::vec3>& _contact_points, const glm::vec3& _normal) const;

    public:
        LPhys::Intersection_Data collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, const Border& _border_1,
                                                           const LDS::Vector<Border>& _polygons_borders_cache_1,
                                                           const Polygon_Holder_Base* _polygon_holder_2, const Border& _border_2,
                                                           const LDS::Vector<Border>& _polygons_borders_cache_2) const override;

    };

}
