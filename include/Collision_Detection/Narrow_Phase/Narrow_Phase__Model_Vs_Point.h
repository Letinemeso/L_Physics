#pragma once

#include <Collision_Detection/Narrow_Phase/Narrow_Phase_Interface.h>


namespace LPhys
{

    class Narrow_Phase__Model_Vs_Point : public Narrow_Phase_Interface
    {
    private:
        //        LEti::Geometry::Simple_Intersection_Data collision__static_vs_point(const Physics_Module_2D& _static, const glm::vec3& _point, const Narrowest_Phase_Interface* _cd) const;

    public:
        void update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions) override;

    };

}
