#pragma once

#include <Collision_Detection/Intersection_Data.h>
#include <Physical_Models/Physical_Model.h>


namespace LPhys
{

    class Primitives_Intersection_Detector
    {
    public:
        virtual ~Primitives_Intersection_Detector() { }

    public:
        virtual LPhys::Intersection_Data collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, const Polygon_Holder_Base* _polygon_holder_2) const = 0;

    };

}
