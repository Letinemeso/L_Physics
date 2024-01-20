#include <Collision_Resolution/Collision_Resolver.h>

using namespace LPhys;


Collision_Resolver::Collision_Resolver()
{

}

Collision_Resolver::~Collision_Resolver()
{
	clear_resolutions();
}



void Collision_Resolver::add_resolution(const std::string& _type_history_1, const std::string& _type_history_2, Collision_Resolution_Interface *_resolution)
{
    L_ASSERT(_resolution);
    L_ASSERT(_type_history_1.size() > 0);
    L_ASSERT(_type_history_2.size() > 0);

    m_registred_resolutions.push_back(_resolution);

    std::string resolution_key = _type_history_1 + _type_history_2;
    L_ASSERT(!m_resolutions_map.find(resolution_key).is_ok());
    m_resolutions_map.insert((std::string&&)resolution_key, _resolution);

    if(_type_history_1 == _type_history_2)
        return;

    resolution_key = _type_history_2 + _type_history_1;  //  reverse
    L_ASSERT(!m_resolutions_map.find(resolution_key).is_ok());
    m_resolutions_map.insert((std::string&&)resolution_key, _resolution);
}

void Collision_Resolver::clear_resolutions()
{
    for(auto it = m_registred_resolutions.begin(); !it.end_reached(); ++it)
		delete *it;

    m_registred_resolutions.clear();
    m_resolutions_map.clear();
}



void Collision_Resolver::resolve_single(const Intersection_Data &_id, float _dt) const
{
    std::string resolution_key = _id.first->get_actual_history() + _id.second->get_actual_history();

    auto it = m_resolutions_map.find(resolution_key);
    L_ASSERT(it.is_ok());

    Collision_Resolution_Interface* resolution = *it;

    resolution->resolve(_id, _dt);

    _id.first->on_collision(_id.second);
    _id.second->on_collision(_id.first);
}

void Collision_Resolver::resolve_all(const LDS::List<Intersection_Data>& _ids, float _dt) const
{
    for(LDS::List<Intersection_Data>::Const_Iterator it = _ids.begin(); !it.end_reached(); ++it)
        resolve_single(*it, _dt);
}
