#include <Collision_Detection/Primitives/SAT_Models_Intersection_3D.h>

#include <limits>

#include <Collision_Detection/Primitives/Polygon_VS_Ray_Intersection.h>

using namespace LPhys;


namespace LPhys
{

    struct MinMax_Pair
    {
        float min = std::numeric_limits<float>::max();
        float max = std::numeric_limits<float>::min();
    };

    MinMax_Pair project_polygon(const Polygon& _polygon, const glm::vec3& _axis)
    {
        MinMax_Pair result;

        for(unsigned int i = 0; i < 3; ++i)
        {
            float projection = LEti::Math::dot_product(_polygon[i], _axis);
            result.min = std::min(result.min, projection);
            result.max = std::max(result.max, projection);
        }

        return result;
    }

    bool overlap_on_axis(const Polygon& _polygon_1, const Polygon& _polygon_2, const glm::vec3& _axis)
    {
        MinMax_Pair pair_1 = project_polygon(_polygon_1, _axis);
        MinMax_Pair pair_2 = project_polygon(_polygon_2, _axis);

        return !(pair_1.max < pair_2.min || pair_2.max < pair_1.min);
    }

    struct Polygons_Intersection_Data
    {
        bool intersection = false;
        glm::vec3 point;
        glm::vec3 normal;
        float depth = 0.0f;
    };

    Polygons_Intersection_Data check_triangles_intersection(const Polygon& _first, const Polygon& _second)
    {
        Polygons_Intersection_Data result;

        LDS::Vector<glm::vec3> test_axes(10);

        auto construct_and_push_axis = [&test_axes](const glm::vec3& _vec_1, const glm::vec3& _vec_2)
        {
            glm::vec3 axis = LEti::Math::cross_product(_vec_1, _vec_2);
            if(LEti::Math::vector_length_squared(axis) < 0.0001f)
                return;

            LEti::Math::shrink_vector_to_1(axis);
            test_axes.push(axis);
        };

        construct_and_push_axis(_first[0] - _first[1], _first[2] - _first[1]);
        construct_and_push_axis(_second[0] - _second[1], _second[2] - _second[1]);

        for(unsigned int i_1 = 0; i_1 < 3; ++i_1)
        {
            glm::vec3 first_edge = _first[i_1 + 1] - _first[i_1];
            for(unsigned int i_2 = 0; i_2 < 3; ++i_2)
            {
                glm::vec3 second_edge = _second[i_2 + 1] - _second[i_2];
                construct_and_push_axis(first_edge, second_edge);
                construct_and_push_axis(second_edge, first_edge);
            }
        }

        float min_overlap = std::numeric_limits<float>::max();
        glm::vec3 min_axis;

        for(unsigned int i = 0; i < test_axes.size(); ++i)
        {
            const glm::vec3& axis = test_axes[i];

            if (!overlap_on_axis(_first, _second, axis))
                return result;

            MinMax_Pair pair_1 = project_polygon(_first, axis);
            MinMax_Pair pair_2 = project_polygon(_second, axis);

            float overlap_1 = pair_1.max - pair_2.min;
            float overlap_2 = pair_2.max - pair_1.min;

            float overlap = std::min(overlap_1, overlap_2);
            if (overlap < min_overlap)
            {
                min_overlap = overlap;
                min_axis = axis;
            }
        }

        result.intersection = true;
        result.depth = min_overlap;
        result.normal = min_axis;

        result.point = { 0.0f, 0.0f, 0.0f };
        unsigned int points_amount = 0;

        for(unsigned int i = 0; i < 3; ++i)
        {
            Polygon_VS_Ray_Intersection_Data id = segment_intersects_polygon(_first[i], _first[i + 1], _second);
            if(!id.intersection)
                continue;

            result.point += id.point;
            ++points_amount;
        }

        if(points_amount == 0)
            result.point = (_first.center() + _second.center()) * 0.5f;
        else
            result.point /= (float)points_amount;

        return result;
    }
}



LPhys::Intersection_Data SAT_Models_Intersection_3D::collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _pols_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _pols_amount_2) const
{
    glm::vec3 result_point = { 0.0f, 0.0f, 0.0f };
    glm::vec3 result_push_out_vector = { 0.0f, 0.0f, 0.0f };
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

            glm::vec3 push_out_vector = id.normal * id.depth;
            for(unsigned int i = 0; i < 3; ++i)
            {
                if(fabsf(result_push_out_vector[i]) < fabsf(push_out_vector[i]))
                    result_push_out_vector[i] = push_out_vector[i];
            }
        }
    }

    if(intersections_amount == 0)
        return {};

    if(LEti::Math::vector_length_squared(result_push_out_vector) < 0.0001f)
        return {};

    result_point /= (float)intersections_amount;

    Intersection_Data result;
    result.type = Intersection_Data::Type::intersection;
    result.depth = LEti::Math::vector_length(result_push_out_vector);
    LEti::Math::shrink_vector_to_1(result_push_out_vector);
    result.normal = -result_push_out_vector;
    result.point = result_point;

    return result;
}
