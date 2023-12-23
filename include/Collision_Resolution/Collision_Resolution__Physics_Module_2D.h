#pragma once

#include <Collision_Resolution/Collision_Resolver.h>
#include <Modules/Physics_Module_2D.h>


namespace LPhys
{

    class Collision_Resolution__Physics_Module_2D : public Collision_Resolution_Interface
    {
    public:
        bool resolve(const Intersection_Data &_id, float _dt = 0.0f) override;

    };

}
