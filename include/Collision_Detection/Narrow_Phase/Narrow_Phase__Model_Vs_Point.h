#pragma once

#include <Collision_Detection/Narrow_Phase/Narrow_Phase_Interface.h>
#include <Collision_Detection/Primitives/Polygon_Vs_Point_Intersection.h>
#include <Modules/Physics_Module_2D.h>
#include <Modules/Physics_Module__Point.h>


namespace LPhys
{

    class Narrow_Phase__Model_Vs_Point : public Narrow_Phase_Interface
    {
    private:
        struct Colliding_Modules
        {
            const Physics_Module_2D* model = nullptr;
            const Physics_Module__Point* point = nullptr;

            inline operator bool() const { return model && point; }
        };

    private:
        Colliding_Modules M_cast_modules(const Broad_Phase_Interface::Colliding_Pair& _maybe_modules) const;
        Intersection_Data M_check_model_vs_point_intersection(const Colliding_Modules& _modules);

    public:
        void update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions) override;

    };

}
