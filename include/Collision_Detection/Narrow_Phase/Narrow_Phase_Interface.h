#pragma once

#include <Collision_Detection/Intersection_Data.h>
#include <Collision_Detection/Broad_Phase/Broad_Phase_Interface.h>
#include <Modules/Physics_Module_2D.h>

namespace LPhys
{

    class Narrow_Phase_Interface
    {
    public:
        using Collision_Data_List = LDS::List<Intersection_Data>;

        Collision_Data_List m_collisions;

    public:
        Narrow_Phase_Interface();
        virtual ~Narrow_Phase_Interface();

    public:
        virtual void update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions) = 0;

    public:
        inline const Collision_Data_List& get_collisions() const { return m_collisions; }

    };

}
