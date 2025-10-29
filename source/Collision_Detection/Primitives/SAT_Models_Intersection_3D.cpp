#include <Collision_Detection/Primitives/SAT_Models_Intersection_3D.h>

#include <limits>

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

    Polygons_Intersection_Data check_triangles_intersection(const Polygon& _first, const Polygon& _second)
    {
        glm::vec3 second_normal = LEti::Math::cross_product(_second[2] - _second[1], _second[0] - _second[1]);
        LEti::Math::shrink_vector_to_1(second_normal);

        float min_depth = std::numeric_limits<float>::max();
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
            if(segment_start_proj < 0.0f)
                depth = fabsf(segment_start_proj);
            else
                depth = fabsf(segment_end_proj);

            if(min_depth > depth)
                min_depth = depth;

            ++found_intersections;
        }

        if(found_intersections == 0)
            return {};

        Polygons_Intersection_Data result;
        result.intersection = true;
        result.point = median_intersection_point / (float)found_intersections;
        result.normal = second_normal;
        result.depth = min_depth;

        return result;
    }

}



LPhys::Intersection_Data SAT_Models_Intersection_3D::collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _pols_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _pols_amount_2) const
{
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

    if(intersections_amount == 0)
        return {};

    if(LEti::Math::vector_length_squared(result_push_out_vector) < 0.0001f)
        return {};

    result_point /= (float)intersections_amount;

    Intersection_Data result;
    result.type = Intersection_Data::Type::intersection;
    result.depth = min_depth;
    result.normal = result_push_out_vector;
    result.point = result_point;

    return result;
}
