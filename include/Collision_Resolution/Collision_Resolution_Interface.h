#pragma once

#include <Collision_Detection/Intersection_Data.h>


namespace LPhys
{

    class Collision_Resolution_Interface
    {
    public:
        Collision_Resolution_Interface() { }
        virtual ~Collision_Resolution_Interface() { }

    public:
        virtual void resolve(const Intersection_Data& _id, float _dt) = 0;

    };

}
