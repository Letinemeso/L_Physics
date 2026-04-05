#pragma once

#include <Collision_Detection/Intersection_Data.h>


namespace LPhys
{

    class Collision_Resolution
    {
    public:
        Collision_Resolution() { }
        virtual ~Collision_Resolution() { }

    public:
        virtual void on_before_pass() { }
        virtual bool resolve(const Intersection_Data& _id, float _dt) = 0;

    };

}
