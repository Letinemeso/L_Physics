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

    Polygons_Intersection_Data check_triangles_intersection(const Polygon& _first, const Polygon& _second)
    {
        glm::vec3 first_normal = LEti::Math::cross_product(_first[2] - _first[1], _first[0] - _first[1]);
        glm::vec3 second_normal = LEti::Math::cross_product(_second[2] - _second[1], _second[0] - _second[1]);
        LEti::Math::shrink_vector_to_1(first_normal);
        LEti::Math::shrink_vector_to_1(second_normal);

        float normals_dot = LEti::Math::dot_product(first_normal, second_normal);
        if(normals_dot > 0.0f)
            return {};

        LDS::Vector<glm::vec3> axes(8);
        axes.push(first_normal);
        axes.push(second_normal);

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
                axes.push(axis);
            }
        }

        Polygons_Intersection_Data result;
        result.depth = std::numeric_limits<float>::max();

        for(unsigned int i = 0; i < axes.size(); ++i)
        {
            const glm::vec3& axis = axes[i];

            Axis_Intersection id = check_intersection_on_axis(axis, _first, _second);
            if(!id.intersection)
                return result;

            if(id.depth >= result.depth)
                continue;

            result.depth = id.depth;
            result.normal = axis;
            if(id.flip_normal)
                result.normal *= -1.0f;
        }

        Polygons_Intersection_Data id = calculate_intersection_point(_first, _second);
        if(!id.intersection)
            return {};

        result.point = id.point;
        result.intersection = true;

        return result;
    }

}



LPhys::Intersection_Data SAT_Models_Intersection_3D::collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _pols_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _pols_amount_2) const
{
    glm::vec3 result_point = { 0.0f, 0.0f, 0.0f };
    glm::vec3 result_push_out_vector = { 0.0f, 0.0f, 0.0f };
    float total_depth = 0.0f;
    unsigned int intersections_amount = 0;

    for(unsigned int i_1 = 0; i_1 < _pols_amount_1; ++i_1)
    {
        for(unsigned int i_2 = 0; i_2 < _pols_amount_2; ++i_2)
        {
            const Polygon& polygon_1 = *_polygon_holder_1->get_polygon(i_1);
            const Polygon& polygon_2 = *_polygon_holder_2->get_polygon(i_2);

            Polygons_Intersection_Data id = check_triangles_intersection(polygon_1, polygon_2);

            if(!id.intersection)
                continue;

            ++intersections_amount;

            result_point += id.point;

            glm::vec3 push_out_vector = -id.normal * id.depth;

            total_depth += id.depth;
            result_push_out_vector += push_out_vector;
        }
    }

    if(intersections_amount == 0 || total_depth < 1e-7f)
        return {};

    result_point /= (float)intersections_amount;

    LEti::Math::shrink_vector_to_1(result_push_out_vector);

    Intersection_Data result;
    result.type = Intersection_Data::Type::intersection;
    result.depth = total_depth / (float)intersections_amount;
    if(result.depth < 1e-7f)
        result.depth *= 1.1f;
    result.normal = result_push_out_vector;
    result.point = result_point;

    return result;
}
