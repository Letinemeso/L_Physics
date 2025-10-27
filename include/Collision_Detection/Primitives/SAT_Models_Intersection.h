#pragma once

#include <vec3.hpp>

#include <Data_Structures/List.h>

#include <Math_Stuff.h>

#include <Collision_Detection/Primitives/Primitives_Intersection_Detector.h>


namespace LPhys
{

    class SAT_Models_Intersection : public Primitives_Intersection_Detector
    {
    public:
        SAT_Models_Intersection() {}
        ~SAT_Models_Intersection() {}

    private:
        struct Intersection_Data
        {
            bool intersection = false;
            float min_dist = -1.0f;
            glm::vec3 min_dist_axis;
        };

        struct MinMax_Pair
        {
            float min = 0.0f;
            float max = 0.0f;
        };

    private:
        void M_rotate_2D_vector_perpendicular(glm::vec3& _vec) const;

        MinMax_Pair M_get_minmax_projections(const glm::vec3& _axis, const Polygon& _pol) const;
        float M_point_to_segment_distance(const glm::vec3& _point, const glm::vec3& _seg_start, const glm::vec3& _seg_end) const;

        Intersection_Data M_polygons_collision(const Polygon& _first, const Polygon& _second) const;

        float M_smallest_point_to_polygon_distance(const glm::vec3& _point, const Polygon& _pol) const;
        LDS::List<glm::vec3> M_points_of_contact(const Polygon_Holder_Base* _f_pols, unsigned int _f_count, const Polygon_Holder_Base* _s_pols, unsigned int _s_count) const;

    public:
        LPhys::Intersection_Data collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _pols_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _pols_amount_2) const override;
    };

}
