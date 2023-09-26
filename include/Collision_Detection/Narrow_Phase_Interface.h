#ifndef __NARROW_PHASE_INTERFACE
#define __NARROW_PHASE_INTERFACE

#include <Collision_Detection/Broad_Phase_Interface.h>
#include <Collision_Detection/Narrowest_Phase_Interface.h>
#include <Modules/Physics_Module_2D.h>

namespace LPhys
{

class Narrow_Phase_Interface
{
public:
    using Collision_Data_List__Models = LDS::List<Intersection_Data>;
    using Collision_Data_List__Points = LDS::List<Intersection_Data>;

    Collision_Data_List__Models m_collisions__models;
    Collision_Data_List__Points m_collisions__points;

public:
    Narrow_Phase_Interface();
    virtual ~Narrow_Phase_Interface();

public:
    virtual void update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions__models, const Broad_Phase_Interface::Colliding_Point_And_Object_List& _possible_collisions__pointsnp, const Narrowest_Phase_Interface* _cd) = 0;

    inline const Collision_Data_List__Models& get_collisions__models() const { return m_collisions__models; }
    inline const Collision_Data_List__Points& get_collisions__points() const { return m_collisions__points; }

};

}


#endif // __NARROW_PHASE_INTERFACE
