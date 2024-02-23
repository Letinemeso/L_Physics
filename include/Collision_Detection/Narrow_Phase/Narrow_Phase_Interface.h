#pragma once

#include <Collision_Detection/Broad_Phase/Broad_Phase_Interface.h>
#include <Collision_Detection/Narrowest_Phase/Narrowest_Phase_Interface.h>
#include <Modules/Physics_Module_2D.h>

namespace LPhys
{

    class Narrow_Phase_Interface
    {
    public:
        using Collision_Data_List__Models = LDS::List<Intersection_Data>;

        Collision_Data_List__Models m_collisions__models;

    public:
        Narrow_Phase_Interface();
        virtual ~Narrow_Phase_Interface();

    public:
        virtual void update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions__models, const Narrowest_Phase_Interface* _cd) = 0;

        inline const Collision_Data_List__Models& get_collisions__models() const { return m_collisions__models; }

    };

}
