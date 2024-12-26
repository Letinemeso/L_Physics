#include <Collision_Resolution/Collision_Resolver.h>

using namespace LPhys;


Collision_Resolver::Collision_Resolver()
{

}

Collision_Resolver::~Collision_Resolver()
{
    delete m_resolution;
}



void Collision_Resolver::resolve_single(const Intersection_Data &_id, float _dt) const
{
    m_resolution->resolve(_id, _dt);

    _id.first->on_collision(_id.second);
    _id.second->on_collision(_id.first);
}

void Collision_Resolver::resolve_all(const LDS::List<Intersection_Data>& _ids, float _dt) const
{
    for(LDS::List<Intersection_Data>::Const_Iterator it = _ids.begin(); !it.end_reached(); ++it)
        resolve_single(*it, _dt);

    for(LDS::List<Intersection_Data>::Const_Iterator it = _ids.begin(); !it.end_reached(); ++it)
    {
        const Intersection_Data& id = *it;

        id.first->apply_data_after_collisions();
        id.second->apply_data_after_collisions();
    }
}
