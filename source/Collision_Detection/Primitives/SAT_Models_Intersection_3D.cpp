#include <Collision_Detection/Primitives/SAT_Models_Intersection_3D.h>

#include <limits>

#include <Data_Structures/Vector.h>

#include <Collision_Detection/Primitives/Polygon_VS_Ray_Intersection.h>

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
            constexpr float tolerance = 1e-6f;
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
            float projection = LEti::Math::dot_product(_first[i], _axis);
            first_pair.add_value(projection);
        }
        for(unsigned int i = 0; i < 3; ++i)
        {
            float projection = LEti::Math::dot_product(_second[i], _axis);
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

    Polygons_Intersection_Data check_triangles_intersection(const Polygon& _first, const Polygon& _second, float _min_plane_edge_difference)
    {
        glm::vec3 first_normal = LEti::Math::cross_product(_first[2] - _first[1], _first[0] - _first[1]);
        glm::vec3 second_normal = LEti::Math::cross_product(_second[2] - _second[1], _second[0] - _second[1]);
        LEti::Math::shrink_vector_to_1(first_normal);
        LEti::Math::shrink_vector_to_1(second_normal);

        float normals_dot = LEti::Math::dot_product(first_normal, second_normal);
        if(normals_dot > 0.0f)
            return {};

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

                glm::vec3 axis = LEti::Math::cross_product(f_edge, s_edge);
                if(LEti::Math::vector_length_squared(axis) < 1e-6f)
                    continue;

                LEti::Math::shrink_vector_to_1(axis);
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

        if(plane_id.depth < edge_id.depth)
            result = plane_id;
        else
        {
            float plane_edge_depth_difference = plane_id.depth - edge_id.depth;
            if(plane_edge_depth_difference < _min_plane_edge_difference)
                result = plane_id;
            else
                result = edge_id;
        }

        Polygons_Intersection_Data id = calculate_intersection_point(_first, _second);
        if(!id.intersection)
            return {};

        result.point = id.point;
        result.intersection = true;

        return result;
    }


    glm::vec3 combine_push_out_vectors(const glm::vec3& _first, const glm::vec3& _second)
    {
        glm::vec3 result;
        for(unsigned int i = 0; i < 3; ++i)
        {
            if(fabsf(_first[i]) > fabsf(_second[i]))
                result[i] = _first[i];
            else
                result[i] = _second[i];
        }
        return result;
    }


    struct Common_Intersection_Data
    {
        glm::vec3 point = { 0.0f, 0.0f, 0.0f };
        glm::vec3 push_out_vector = { 0.0f, 0.0f, 0.0f };
        unsigned int intersections_amount = 0;

        void operator+=(const Common_Intersection_Data& _other)
        {
            point += _other.point;
            push_out_vector = combine_push_out_vectors(push_out_vector, _other.push_out_vector);
            intersections_amount += _other.intersections_amount;
        }
    };


    Common_Intersection_Data calculate_common_intersection(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _polygons_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _polygons_amount_2, float _min_plane_edge_difference)
    {
        Common_Intersection_Data result;

        for(unsigned int i_1 = 0; i_1 < _polygons_amount_1; ++i_1)
        {
            for(unsigned int i_2 = 0; i_2 < _polygons_amount_2; ++i_2)
            {
                const Polygon& polygon_1 = *_polygon_holder_1->get_polygon(i_1);
                const Polygon& polygon_2 = *_polygon_holder_2->get_polygon(i_2);

                Polygons_Intersection_Data id = check_triangles_intersection(polygon_1, polygon_2, _min_plane_edge_difference);

                if(!id.intersection)
                    continue;

                ++result.intersections_amount;

                result.point += id.point;

                glm::vec3 push_out_vector = -id.normal * id.depth;

                result.push_out_vector = combine_push_out_vectors(result.push_out_vector, push_out_vector);
            }
        }

        return result;
    }


    using Polygons_Vec = LDS::Vector<const Polygon*>;

    Common_Intersection_Data calculate_common_intersection(const Polygons_Vec& _p1, const Polygons_Vec& _p2, float _min_plane_edge_difference)
    {
        Common_Intersection_Data result;

        for(unsigned int i_1 = 0; i_1 < _p1.size(); ++i_1)
        {
            for(unsigned int i_2 = 0; i_2 < _p2.size(); ++i_2)
            {
                const Polygon& polygon_1 = *_p1[i_1];
                const Polygon& polygon_2 = *_p2[i_2];

                Polygons_Intersection_Data id = check_triangles_intersection(polygon_1, polygon_2, _min_plane_edge_difference);

                if(!id.intersection)
                    continue;

                ++result.intersections_amount;

                result.point += id.point;

                glm::vec3 push_out_vector = -id.normal * id.depth;

                result.push_out_vector = combine_push_out_vectors(result.push_out_vector, push_out_vector);
            }
        }

        return result;
    }

    struct Polygons_In_Area
    {
        Border area;
        Polygons_Vec polygons_1;
        Polygons_Vec polygons_2;
    };

    Polygons_In_Area find_polygons_in_area(const Polygons_Vec& _p1, const Polygons_Vec& _p2, const Border& _border)
    {
        Polygons_In_Area result;
        result.polygons_1.resize(_p1.size());
        result.polygons_2.resize(_p2.size());

        for(unsigned int p_i = 0; p_i < _p1.size(); ++p_i)
        {
            const Polygon& polygon = *_p1[p_i];

            bool inside = false;
            for(unsigned int i = 0; i < 3; ++i)
            {
                if( _border.point_is_inside(polygon[i]) )
                {
                    inside = true;
                    break;
                }
            }

            if(!inside)
                continue;

            result.polygons_1.push(&polygon);

            for(unsigned int i = 0; i < 3; ++i)
                result.area.consider_point(polygon[i]);
        }

        for(unsigned int p_i = 0; p_i < _p2.size(); ++p_i)
        {
            const Polygon& polygon = *_p2[p_i];

            bool inside = false;
            for(unsigned int i = 0; i < 3; ++i)
            {
                if( _border.point_is_inside(polygon[i]) )
                {
                    inside = true;
                    break;
                }
            }

            if(!inside)
                continue;

            result.polygons_2.push(&polygon);

            for(unsigned int i = 0; i < 3; ++i)
                result.area.consider_point(polygon[i]);
        }

        return result;
    }

    bool same_polygons(const Polygons_Vec& _v1, const Polygons_Vec& _v2)
    {
        if(_v1.size() != _v2.size())
            return false;

        for(unsigned int i = 0; i < _v1.size(); ++i)
        {
            if(_v1[i] != _v2[i])
                return false;
        }

        return true;
    }

    Common_Intersection_Data calculate_common_intersection_in_area(const Polygons_Vec& _p1, const Polygons_Vec& _p2, const Border& _border, unsigned int _min_polygons, unsigned int _repeated_depth_limit, float _min_plane_edge_difference)
    {
        if(_p1.size() == 0 || _p2.size() == 0)
            return {};

        if(_repeated_depth_limit >= 3)
            return calculate_common_intersection(_p1, _p2, _min_plane_edge_difference);

        if(_p1.size() < _min_polygons && _p2.size() < _min_polygons)
            return calculate_common_intersection(_p1, _p2, _min_plane_edge_difference);

        Border b1 = _border;
        Border b2 = _border;

        if(_border.size().x > _border.size().y && _border.size().x > _border.size().z)
        {
            glm::vec3 diff = {_border.size().x * 0.5f, 0.0f, 0.0f};

            b1.modify_size(-diff);
            b2.modify_size(-diff);
            b2.modify_offset(diff);
        }
        else if(_border.size().y > _border.size().z)
        {
            glm::vec3 diff = {0.0f, _border.size().y * 0.5f, 0.0f};

            b1.modify_size(-diff);
            b2.modify_size(-diff);
            b2.modify_offset(diff);
        }
        else
        {
            glm::vec3 diff = {0.0f, 0.0f, _border.size().z * 0.5f};

            b1.modify_size(-diff);
            b2.modify_size(-diff);
            b2.modify_offset(diff);
        }

        Polygons_In_Area p1 = find_polygons_in_area(_p1, _p2, b1);
        Polygons_In_Area p2 = find_polygons_in_area(_p1, _p2, b2);

        unsigned int counter_1 = 0;
        if(same_polygons(p1.polygons_1, _p1) && same_polygons(p1.polygons_2, _p2))
            counter_1 = _repeated_depth_limit + 1;

        unsigned int counter_2 = 0;
        if(same_polygons(p2.polygons_1, _p1) && same_polygons(p2.polygons_2, _p2))
            counter_2 = _repeated_depth_limit + 1;

        if(same_polygons(p1.polygons_1, p2.polygons_1) && same_polygons(p1.polygons_2, p2.polygons_2))
            return calculate_common_intersection_in_area(p1.polygons_1, p1.polygons_2, p1.area, _min_polygons, counter_1, _min_plane_edge_difference);

        Common_Intersection_Data result = calculate_common_intersection_in_area(p1.polygons_1, p1.polygons_2, p1.area, _min_polygons, counter_1, _min_plane_edge_difference);
        result += calculate_common_intersection_in_area(p2.polygons_1, p2.polygons_2, p2.area, _min_polygons, counter_2, _min_plane_edge_difference);

        return result;
    }

    Common_Intersection_Data calculate_common_intersection_optimized(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _polygons_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _polygons_amount_2, unsigned int _min_polygons, float _min_plane_edge_difference)
    {
        Polygons_Vec polygons_1(_polygons_amount_1);
        Polygons_Vec polygons_2(_polygons_amount_2);

        Border common_area;

        for(unsigned int p_i = 0; p_i < _polygons_amount_1; ++p_i)
        {
            const Polygon& polygon = *_polygon_holder_1->get_polygon(p_i);
            polygons_1.push(&polygon);

            for(unsigned int i = 0; i < 3; ++i)
                common_area.consider_point(polygon[i]);
        }
        for(unsigned int p_i = 0; p_i < _polygons_amount_2; ++p_i)
        {
            const Polygon& polygon = *_polygon_holder_2->get_polygon(p_i);
            polygons_2.push(&polygon);

            for(unsigned int i = 0; i < 3; ++i)
                common_area.consider_point(polygon[i]);
        }

        Common_Intersection_Data result = calculate_common_intersection_in_area(polygons_1, polygons_2, common_area, _min_polygons, 0, _min_plane_edge_difference);
        return result;
    }

}



LPhys::Intersection_Data SAT_Models_Intersection_3D::collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _polygons_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _polygons_amount_2) const
{
    Common_Intersection_Data id;
    if(_polygons_amount_1 < m_min_polygons_for_optimization && _polygons_amount_2 < m_min_polygons_for_optimization)
        id = calculate_common_intersection(_polygon_holder_1, _polygons_amount_1, _polygon_holder_2, _polygons_amount_2, m_min_plane_edge_difference);
    else
        id = calculate_common_intersection_optimized(_polygon_holder_1, _polygons_amount_1, _polygon_holder_2, _polygons_amount_2, m_min_polygons_for_optimization, m_min_plane_edge_difference);

    float push_out_vector_length = LEti::Math::vector_length(id.push_out_vector);

    if(id.intersections_amount == 0 || push_out_vector_length < 1e-7f)
        return {};

    // LEti::Math::shrink_vector_to_1(id.push_out_vector);

    Intersection_Data result;
    result.type = Intersection_Data::Type::intersection;
    result.depth = push_out_vector_length;
    if(result.depth < 1e-7f)
        result.depth *= 1.1f;
    result.normal = id.push_out_vector / push_out_vector_length;
    result.point = id.point / (float)id.intersections_amount;

    return result;
}
