#include <Collision_Detection/Primitives/SAT_Models_Intersection_3D.h>

#include <algorithm>
#include <limits>

#include <Collision_Detection/Primitives/Polygon_VS_Ray_Intersection.h>

using namespace LPhys;


namespace LPhys
{

    struct Segments_Closest_Points_Data
    {
        bool found = false;
        glm::vec3 point_1;
        glm::vec3 point_2;
    };

    Segments_Closest_Points_Data find_closest_points_between_segments(const glm::vec3& _first_start, const glm::vec3& _first_end, const glm::vec3& _second_start, const glm::vec3& _second_end)
    {
        constexpr float EPSILON = 1e-6f;

        glm::vec3 ab = _first_end - _first_start;
        glm::vec3 cd = _second_end - _second_start;
        glm::vec3 ac = _second_start - _first_start;

        float ab_ab = LEti::Math::dot_product(ab, ab);
        float cd_cd = LEti::Math::dot_product(cd, cd);
        float ab_cd = LEti::Math::dot_product(ab, cd);
        float ab_ac = LEti::Math::dot_product(ab, ac);
        float cd_ac = LEti::Math::dot_product(cd, ac);

        float denominator = ab_cd * ab_cd - ab_ab * cd_cd;

        float ratio_1 = 0.0f;
        float ratio_2 = 0.0f;


        if (fabsf(denominator) <= EPSILON)
            return {};

        denominator = 1.0f / denominator;
        ratio_1 = (ab_cd * cd_ac - cd_cd * ab_ac) * denominator;
        ratio_2 = (ab_ab * cd_ac - ab_cd * ab_ac) * denominator;

        ratio_1 = std::clamp(ratio_1, 0.0f, 1.0f);
        ratio_2 = std::clamp(ratio_2, 0.0f, 1.0f);

        Segments_Closest_Points_Data result;
        result.found = true;

        result.point_1 = _first_start + ratio_1 * (_first_end - _first_start);
        result.point_2 = _second_start + ratio_2 * (_second_end - _second_start);

        return result;
    }

    struct Polygons_Intersection_Data
    {
        bool intersection = false;
        glm::vec3 point;
        glm::vec3 normal;
        float depth = 0.0f;
    };

    Segments_Closest_Points_Data calculate_minimal_edge_to_edge_intersection(const glm::vec3& _segment_start, const glm::vec3& _segment_end, const Polygon& _polygon, float _min_distance)
    {
        Segments_Closest_Points_Data result;

        float min_distance_squared = _min_distance * _min_distance;

        for(unsigned int i = 0; i < 3; ++i)
        {
            Segments_Closest_Points_Data id = find_closest_points_between_segments(_segment_start, _segment_end, _polygon[i], _polygon[i + 1]);
            if(!id.found)
                continue;

            float distance_squared = LEti::Math::vector_length_squared(id.point_2 - id.point_1);
            if(distance_squared >= min_distance_squared)
                continue;

            if(distance_squared < 1e-11f)
            {
                std::cout << "skipping edge-edge collision with distance: " << distance_squared << std::endl;
                continue;
            }

            min_distance_squared = distance_squared;
            result = id;
        }

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

        float min_depth = std::numeric_limits<float>::max();
        glm::vec3 min_depth_axis;
        glm::vec3 median_intersection_point = {0.0f, 0.0f, 0.0f};

        unsigned int found_intersections = 0;
        for(unsigned int i = 0; i < 3; ++i)
        {
            Polygon_VS_Ray_Intersection_Data id = segment_intersects_polygon(_first[i], _first[i + 1], _second);
            if(!id.intersection)
                continue;

            const glm::vec3& intersection_point = id.point;
            median_intersection_point += intersection_point;

            float ip_projection = LEti::Math::dot_product(intersection_point, second_normal);
            float segment_start_proj = LEti::Math::dot_product(_first[i], second_normal) - ip_projection;
            float segment_end_proj = LEti::Math::dot_product(_first[i + 1], second_normal) - ip_projection;

            float depth;
            if(segment_start_proj < 0.00001f)
                depth = fabsf(segment_start_proj);
            else
                depth = fabsf(segment_end_proj);

            glm::vec3 axis;

            Segments_Closest_Points_Data edge_to_edge_id = calculate_minimal_edge_to_edge_intersection(_first[i], _first[i + 1], _second, depth);
            if(edge_to_edge_id.found)
            {
                // glm::vec3 edge = _first[i + 1] - _first[i];
                // glm::vec3 push_out_axis = LEti::Math::rotate_vector(edge, first_normal, -LEti::Math::HALF_PI);
                // LEti::Math::shrink_vector_to_1(push_out_axis);

                // glm::vec3 point_1_projected = LEti::Math::calculate_projection(edge_to_edge_id.point_1, push_out_axis);
                // glm::vec3 point_2_projected = LEti::Math::calculate_projection(edge_to_edge_id.point_2, push_out_axis);

                // glm::vec3 push_out_vector = point_1_projected - point_2_projected;
                glm::vec3 push_out_vector = edge_to_edge_id.point_1 - edge_to_edge_id.point_2;
                float length = LEti::Math::vector_length(push_out_vector);
                depth = length * 1.1f;
                // axis = push_out_axis;
                axis = push_out_vector / length;

                // std::cout << "found closer edge-edge contact" << std::endl;
            }
            else
            {
                axis = second_normal;
            }

            if(min_depth > depth)
            {
                min_depth = depth;
                min_depth_axis = axis;
            }

            ++found_intersections;
        }

        if(found_intersections == 0)
            return {};

        Polygons_Intersection_Data result;
        result.intersection = true;
        result.point = median_intersection_point / (float)found_intersections;
        result.normal = second_normal;
        result.depth = min_depth;

        std::cout << "found collision, dot between normals: " << normals_dot << std::endl;

        return result;
    }

}



LPhys::Intersection_Data SAT_Models_Intersection_3D::collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _pols_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _pols_amount_2) const
{
    /*
    glm::vec3 result_point = { 0.0f, 0.0f, 0.0f };
    glm::vec3 result_push_out_vector = { 0.0f, 0.0f, 0.0f };
    float min_depth = std::numeric_limits<float>::max();
    unsigned int intersections_amount = 0;

    for(unsigned int i_1 = 0; i_1 < _pols_amount_1; ++i_1)
    {
        for(unsigned int i_2 = 0; i_2 < _pols_amount_2; ++i_2)
        {
            const Polygon& polygon_1 = *_polygon_holder_1->get_polygon(i_1);
            const Polygon& polygon_2 = *_polygon_holder_2->get_polygon(i_2);

            Polygons_Intersection_Data id = check_triangles_intersection(polygon_1, polygon_2);
            Polygons_Intersection_Data id_reverse = check_triangles_intersection(polygon_2, polygon_1);

            bool reversed = false;

            if(!id.intersection)
            {
                id = id_reverse;
                reversed = true;
            }

            if(!id.intersection)
                continue;

            if(id_reverse.intersection && id.depth < id_reverse.depth)
            {
                id = id_reverse;
                reversed = true;
            }

            ++intersections_amount;

            result_point += id.point;

            if(min_depth < id.depth)
                continue;

            min_depth = id.depth;
            result_push_out_vector = id.normal;

            if(reversed)
                result_push_out_vector *= -1.0f;
        }
    }
    */

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
            Polygons_Intersection_Data id_reverse = check_triangles_intersection(polygon_2, polygon_1);

            bool reversed = false;

            if(!id.intersection)
            {
                id = id_reverse;
                reversed = true;
            }

            if(!id.intersection)
                continue;

            if(id_reverse.intersection && id.depth < id_reverse.depth)
            {
                id = id_reverse;
                reversed = true;
            }

            ++intersections_amount;

            result_point += id.point;

            glm::vec3 push_out_vector = id.normal * id.depth;
            if(reversed)
                push_out_vector *= -1.0f;

            for(unsigned int i = 0; i < 3; ++i)
            {
                if(fabsf(result_push_out_vector[i]) < fabsf(push_out_vector[i]))
                    result_push_out_vector[i] = push_out_vector[i];
            }
        }
    }

    if(intersections_amount == 0)
        return {};

    if(LEti::Math::vector_length_squared(result_push_out_vector) < 1e-11f)
        return {};

    result_point /= (float)intersections_amount;

    // Intersection_Data result;
    // result.type = Intersection_Data::Type::intersection;
    // result.depth = min_depth;
    // result.normal = result_push_out_vector;
    // result.point = result_point;

    Intersection_Data result;
    result.type = Intersection_Data::Type::intersection;
    result.depth = LEti::Math::vector_length(result_push_out_vector);
    result.normal = result_push_out_vector / result.depth;
    result.point = result_point;

    return result;
}
