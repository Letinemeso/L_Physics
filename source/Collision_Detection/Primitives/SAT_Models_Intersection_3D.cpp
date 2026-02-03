#include <Collision_Detection/Primitives/SAT_Models_Intersection_3D.h>

#include <limits>

#include <Data_Structures/Vector.h>

#include <Collision_Detection/Primitives/Polygon_VS_Ray_Intersection.h>
#include <Collision_Detection/Tools/Bounding_Volume_Hierarchy.h>

using namespace LPhys;


namespace LPhys
{

    struct Polygons_Intersection_Data
    {
        bool intersection = false;
        glm::vec3 point;
        glm::vec3 normal;
        float depth = 0.0f;
    };

    Polygons_Intersection_Data calculate_intersection_point(const Polygon& _first, const Polygon& _second)
    {
        unsigned int intersections_amount = 0;
        glm::vec3 median_intersection_point = {0.0f, 0.0f, 0.0f};

        for(unsigned int i = 0; i < 3; ++i)
        {
            Polygon_VS_Ray_Intersection_Data id = segment_intersects_polygon(_first[i], _first[i + 1], _second);
            if(!id.intersection)
                continue;

            median_intersection_point += id.point;
            ++intersections_amount;
        }

        for(unsigned int i = 0; i < 3; ++i)
        {
            Polygon_VS_Ray_Intersection_Data id = segment_intersects_polygon(_second[i], _second[i + 1], _first);
            if(!id.intersection)
                continue;

            median_intersection_point += id.point;
            ++intersections_amount;
        }

        if(intersections_amount == 0)
            return {};

        Polygons_Intersection_Data result;
        result.intersection = true;
        result.point = median_intersection_point / (float)intersections_amount;

        return result;
    }

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

    Axis_Intersection check_intersection_on_axis(const glm::vec3& _axis, const Polygon& _first, const Polygon& _second)
    {
        MinMax_Pair first_pair;
        MinMax_Pair second_pair;

        for(unsigned int i = 0; i < 3; ++i)
        {
            float projection = LST::Math::dot_product(_first[i], _axis);
            first_pair.add_value(projection);
        }
        for(unsigned int i = 0; i < 3; ++i)
        {
            float projection = LST::Math::dot_product(_second[i], _axis);
            second_pair.add_value(projection);
        }

        if( !(first_pair.value_is_between(second_pair.min) || first_pair.value_is_between(second_pair.max) || (second_pair.value_is_between(first_pair.min) || second_pair.value_is_between(first_pair.max))) )
            return {};

        float dist_1 = first_pair.max - second_pair.min;
        float dist_2 = second_pair.max - first_pair.min;

        Axis_Intersection result;

        if(dist_1 <= 0.0f)
        {
            dist_1 = dist_2;
            result.flip_normal = true;
        }
        if(dist_2 > 0.0f && dist_1 > dist_2)
        {
            dist_1 = dist_2;
            result.flip_normal = true;
        }

        if(dist_1 <= 0.0f)
            return {};

        result.intersection = true;
        result.depth = dist_1;

        return result;
    }

    Polygons_Intersection_Data check_triangles_intersection(const Polygon& _first, const Polygon& _second, float _plane_contact_priority_ratio)
    {
        glm::vec3 first_normal = LST::Math::cross_product(_first[2] - _first[1], _first[0] - _first[1]);
        glm::vec3 second_normal = LST::Math::cross_product(_second[2] - _second[1], _second[0] - _second[1]);

        float normals_dot = LST::Math::dot_product(first_normal, second_normal);
        if(normals_dot > 0.0f)
            return {};

        LST::Math::shrink_vector_to_1(first_normal);
        LST::Math::shrink_vector_to_1(second_normal);

        unsigned int plane_axes_amount = 2;
        glm::vec3 plane_axes[2];
        plane_axes[0] = first_normal;
        plane_axes[1] = second_normal;

        glm::vec3 edge_axes[9];
        unsigned int edge_axes_amount = 0;
        for(unsigned int f_i = 0; f_i < 3; ++f_i)
        {
            glm::vec3 f_edge = _first[f_i + 1] - _first[f_i];
            for(unsigned int s_i = 0; s_i < 3; ++s_i)
            {
                glm::vec3 s_edge = _second[s_i + 1] - _second[s_i];

                glm::vec3 axis = LST::Math::cross_product(f_edge, s_edge);
                if(LST::Math::vector_length_squared(axis) < 1e-9f)
                    continue;

                LST::Math::shrink_vector_to_1(axis);
                edge_axes[edge_axes_amount] = axis;
                ++edge_axes_amount;
            }
        }

        Polygons_Intersection_Data plane_id;
        plane_id.depth = std::numeric_limits<float>::max();

        for(unsigned int i = 0; i < plane_axes_amount; ++i)
        {
            const glm::vec3& axis = plane_axes[i];

            Axis_Intersection id = check_intersection_on_axis(axis, _first, _second);
            if(!id.intersection)
                return plane_id;

            if(id.depth >= plane_id.depth)
                continue;

            plane_id.depth = id.depth;
            plane_id.normal = axis;
            if(id.flip_normal)
                plane_id.normal *= -1.0f;
        }

        Polygons_Intersection_Data edge_id;
        edge_id.depth = std::numeric_limits<float>::max();

        for(unsigned int i = 0; i < edge_axes_amount; ++i)
        {
            const glm::vec3& axis = edge_axes[i];

            Axis_Intersection id = check_intersection_on_axis(axis, _first, _second);
            if(!id.intersection)
                return edge_id;

            if(id.depth >= edge_id.depth)
                continue;

            edge_id.depth = id.depth;
            edge_id.normal = axis;
            if(id.flip_normal)
                edge_id.normal *= -1.0f;
        }

        Polygons_Intersection_Data result;

        float edge_depth_multiplied = edge_id.depth * _plane_contact_priority_ratio;

        if(plane_id.depth < edge_depth_multiplied)
            result = plane_id;
        else
            result = edge_id;

        Polygons_Intersection_Data id = calculate_intersection_point(_first, _second);
        if(!id.intersection)
            return {};

        result.point = id.point;
        result.intersection = true;

        if(LST::Math::dot_product(result.normal, first_normal) < 0.0f)
            result.normal *= -1.0f;

        return result;
    }


    struct Common_Intersection_Data
    {
        glm::vec3 point = { 0.0f, 0.0f, 0.0f };
        glm::vec3 push_out_vector = { 0.0f, 0.0f, 0.0f };
        float total_depth = 0.0f;
        unsigned int intersections_amount = 0;
    };


    Common_Intersection_Data calculate_common_intersection(const Polygon_Holder_Base* _polygon_holder_1,
                                                           const LDS::Vector<Border>& _polygons_borders_cache_1,
                                                           const Polygon_Holder_Base* _polygon_holder_2,
                                                           const LDS::Vector<Border>& _polygons_borders_cache_2,
                                                           float _plane_contact_priority_ratio)
    {
        Common_Intersection_Data result;

        Border border_cache_1;      //  crutch so lambda later can return reference instead of new object
        Border border_cache_2;

        auto get_or_calculate_polygon_border = [](const LDS::Vector<Border>& _polygons_borders_cache, const Polygon& _polygon, unsigned int _polygon_index, Border& _cache)->const Border&
        {
            if(_polygons_borders_cache.size() > 0)
                return _polygons_borders_cache[_polygon_index];

            _cache = _polygon.construct_border();
            return _cache;
        };

        for(unsigned int i_1 = 0; i_1 < _polygon_holder_1->amount(); ++i_1)
        {
            const Polygon& polygon_1 = *_polygon_holder_1->get_polygon(i_1);
            const Border& border_1 = get_or_calculate_polygon_border(_polygons_borders_cache_1, polygon_1, i_1, border_cache_1);

            for(unsigned int i_2 = 0; i_2 < _polygon_holder_2->amount(); ++i_2)
            {
                const Polygon& polygon_2 = *_polygon_holder_2->get_polygon(i_2);
                const Border& border_2 = get_or_calculate_polygon_border(_polygons_borders_cache_2, polygon_2, i_2, border_cache_2);

                if(!border_1.intersects_with(border_2))
                    continue;

                Polygons_Intersection_Data id = check_triangles_intersection(polygon_1, polygon_2, _plane_contact_priority_ratio);

                if(!id.intersection)
                    continue;

                ++result.intersections_amount;

                result.point += id.point;

                glm::vec3 push_out_vector = -id.normal * id.depth;

                result.push_out_vector += push_out_vector;
                result.total_depth += id.depth;
            }
        }

        return result;
    }

    Common_Intersection_Data calculate_common_intersection_optimized(const Polygon_Holder_Base* _polygon_holder_1,
                                                                     const Border& _border_1,
                                                                     const LDS::Vector<Border>& _polygons_borders_cache_1,
                                                                     const Polygon_Holder_Base* _polygon_holder_2,
                                                                     const Border& _border_2,
                                                                     const LDS::Vector<Border>& _polygons_borders_cache_2,
                                                                     float _min_polygons_for_optimization,
                                                                     float _plane_contact_priority_ratio)
    {
        Common_Intersection_Data result;

        Border exclusion_border = _border_1 && _border_2;

        Possible_Colliding_Polygons possible_colliding_polygons = find_possible_colliding_polygons(*_polygon_holder_1, _polygons_borders_cache_1, *_polygon_holder_2, _polygons_borders_cache_2, 3, exclusion_border);

        for(unsigned int i = 0; i < possible_colliding_polygons.size(); ++i)
        {
            const Polygons_Pair& pair = possible_colliding_polygons[i];

            Polygons_Intersection_Data id = check_triangles_intersection(*pair.first, *pair.second, _plane_contact_priority_ratio);

            if(!id.intersection)
                continue;

            ++result.intersections_amount;

            result.point += id.point;

            glm::vec3 push_out_vector = -id.normal * id.depth;

            result.push_out_vector += push_out_vector;
            result.total_depth += id.depth;
        }

        return result;
    }

}



LPhys::Intersection_Data SAT_Models_Intersection_3D::collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1,
                                                                               const Border& _border_1,
                                                                               const LDS::Vector<Border>& _polygons_borders_cache_1,
                                                                               const Polygon_Holder_Base* _polygon_holder_2,
                                                                               const Border& _border_2,
                                                                               const LDS::Vector<Border>& _polygons_borders_cache_2) const
{
    Common_Intersection_Data id;
    if(_polygon_holder_1->amount() < m_min_polygons_for_optimization && _polygon_holder_2->amount() < m_min_polygons_for_optimization)
        id = calculate_common_intersection(_polygon_holder_1, _polygons_borders_cache_1, _polygon_holder_2, _polygons_borders_cache_2, m_plane_contact_priority_ratio);
    else
        id = calculate_common_intersection_optimized(_polygon_holder_1, _border_1, _polygons_borders_cache_1, _polygon_holder_2, _border_2, _polygons_borders_cache_2, m_min_polygons_for_optimization, m_plane_contact_priority_ratio);

    float push_out_vector_length = LST::Math::vector_length(id.push_out_vector);

    if(id.intersections_amount == 0 || push_out_vector_length < 1e-9f)
        return {};

    Intersection_Data result;
    result.intersection = true;
    result.depth = id.total_depth / (float)id.intersections_amount;
    result.normal = id.push_out_vector / push_out_vector_length;
    result.point = id.point / (float)id.intersections_amount;

    return result;
}
