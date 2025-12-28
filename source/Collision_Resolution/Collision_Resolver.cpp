#include <Collision_Resolution/Collision_Resolver.h>

using namespace LPhys;


Collision_Resolver::Collision_Resolver()
{

}

Collision_Resolver::~Collision_Resolver()
{
    clear_resolutions();
}



void Collision_Resolver::clear_resolutions()
{
    for(unsigned int i = 0; i < m_resolutions.size(); ++i)
        delete m_resolutions[i];
    m_resolutions.mark_empty();
}



void Collision_Resolver::resolve_single(const Intersection_Data &_id, float _dt) const
{
    L_ASSERT(m_resolutions.size() > 0);

    for(unsigned int i = 0; i < m_resolutions.size(); ++i)
    {
        Collision_Resolution_Interface* resolution = m_resolutions[i];

        if(!resolution->resolve(_id, _dt))
            continue;

        _id.first->on_collision(_id.second);
        _id.second->on_collision(_id.first);

        break;
    }
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
