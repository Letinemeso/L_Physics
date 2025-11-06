#pragma once

#include <Collision_Detection/Primitives/Primitives_Intersection_Detector.h>


namespace LPhys
{

    class SAT_Models_Intersection_3D : public Primitives_Intersection_Detector
    {
    private:
        unsigned int m_min_polygons_for_optimization = 8;

    public:
        inline void set_min_polygons_for_optimization(unsigned int _value) { m_min_polygons_for_optimization = _value; }

    public:
        LPhys::Intersection_Data collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _polygons_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _polygons_amount_2) const override;

    };

}
