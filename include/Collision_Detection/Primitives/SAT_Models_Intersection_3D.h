#pragma once

#include <Collision_Detection/Primitives/Primitives_Intersection_Detector.h>


namespace LPhys
{

    class SAT_Models_Intersection_3D : public Primitives_Intersection_Detector
    {
    public:
        LPhys::Intersection_Data collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _pols_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _pols_amount_2) const override;

    };

}
