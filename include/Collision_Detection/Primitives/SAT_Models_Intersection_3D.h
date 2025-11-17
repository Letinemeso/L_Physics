#pragma once

#include <Collision_Detection/Primitives/Primitives_Intersection_Detector.h>


namespace LPhys
{

    class SAT_Models_Intersection_3D : public Primitives_Intersection_Detector
    {
    private:
        unsigned int m_min_polygons_for_optimization = 8;   //  if polygons amount on any mesh exceeds this value, optimization algotythm is used (something with space partitioning)
        float m_min_plane_edge_difference = 0.1f;           //  if edge contact has depth difference with plane contact less than this value, plane contact is prioritized

    public:
        inline void set_min_polygons_for_optimization(unsigned int _value) { m_min_polygons_for_optimization = _value; }
        inline void set_min_plane_edge_difference(float _value) { m_min_plane_edge_difference = _value; }

    public:
        LPhys::Intersection_Data collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, const Polygon_Holder_Base* _polygon_holder_2) const override;

    };

}
