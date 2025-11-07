#pragma once

#include <Collision_Resolution/Collision_Resolution_Interface.h>


namespace LPhys
{

    class Collision_Resolution__Physics_Module__Mesh : public Collision_Resolution_Interface
    {
    private:
        bool M_resolve_dynamic_vs_dynamic(const Intersection_Data &_id, float _dt);
        bool M_resolve_dynamic_vs_static(const Intersection_Data &_id, float _dt);

    public:
        bool resolve(const Intersection_Data &_id, float _dt = 0.0f) override;

    };

}
