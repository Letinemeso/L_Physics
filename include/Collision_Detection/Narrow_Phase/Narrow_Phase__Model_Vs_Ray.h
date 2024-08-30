#pragma once

#include <Collision_Detection/Narrow_Phase/Narrow_Phase_Interface.h>
#include <Modules/Physics_Module_2D.h>
#include <Modules/Physics_Module__Ray.h>


namespace LPhys
{

class Narrow_Phase__Model_Vs_Ray : public Narrow_Phase_Interface
{
private:
    struct Colliding_Modules
    {
        const Physics_Module_2D* model = nullptr;
        const Physics_Module__Ray* ray = nullptr;

        inline operator bool() const { return model && ray; }
    };

private:
    float m_tolerance = 0.0001f;

public:
    inline void set_tolerance(float _value) { m_tolerance = _value; }
    inline float tolerance() const { return m_tolerance; }

private:
    Colliding_Modules M_cast_modules(const Broad_Phase_Interface::Colliding_Pair& _maybe_modules) const;
    Intersection_Data M_check_model_vs_ray_intersection(const Colliding_Modules& _modules);

public:
    void update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions) override;

};

}
