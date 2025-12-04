#include <Collision_Detection/Collision_Detector.h>

using namespace LPhys;


Collision_Detector::Collision_Detector()
{

}

Collision_Detector::~Collision_Detector()
{
    delete m_broad_phase;
    delete m_narrow_phase;
}



void Collision_Detector::set_broad_phase(Broad_Phase_Interface* _broad_phase_impl)
{
	delete m_broad_phase;
    m_broad_phase = _broad_phase_impl;
}

void Collision_Detector::set_narrow_phase(Narrow_Phase_Interface* _narrow_phase_impl)
{
    delete m_narrow_phase;
    m_narrow_phase = _narrow_phase_impl;
}



void Collision_Detector::register_module(Physics_Module *_module)
{
    L_DEBUG_FUNC_1ARG([this](const Physics_Module* _module)
    {
        for(auto it = m_registred_modules.begin(); !it.end_reached(); ++it)
        {
            L_ASSERT(*it != _module);
        }
    }, _module);

    m_registred_modules.push_back(_module);
}

void Collision_Detector::register_modules(const Registred_Modules_List& _modules)
{
    for(Registred_Modules_List::Const_Iterator it = _modules.begin(); !it.end_reached(); ++it)
        m_registred_modules.push_back(*it);
}

void Collision_Detector::unregister_module(Physics_Module *_module)
{
    Registred_Modules_List::Iterator it = m_registred_modules.begin();
    while(!it.end_reached())
	{
        if(*it == _module)
            break;
		++it;
	}
    L_ASSERT(!it.end_reached());

    m_registred_modules.erase(it);
}

void Collision_Detector::unregister_all_modules()
{
    m_registred_modules.clear();
}



void Collision_Detector::update()
{
    L_ASSERT(m_broad_phase);
    L_ASSERT(m_narrow_phase);

    m_broad_phase->reset();
    m_broad_phase->add_models(m_registred_modules);
    m_broad_phase->process();

    const Broad_Phase_Interface::Colliding_Pair_List& possible_collisions = m_broad_phase->possible_collisions();

    m_narrow_phase->update(m_broad_phase->possible_collisions());
}

void Collision_Detector::update_with_external_models(const Registred_Modules_List& _external_models)
{
    L_ASSERT(m_broad_phase);
    L_ASSERT(m_narrow_phase);

    m_broad_phase->reset();
    m_broad_phase->add_models(_external_models);
    m_broad_phase->add_models(m_registred_modules);
    m_broad_phase->process();

    const Broad_Phase_Interface::Colliding_Pair_List& possible_collisions = m_broad_phase->possible_collisions();

    m_narrow_phase->update(m_broad_phase->possible_collisions());
}



const Collision_Detector::Intersection_Data_List& Collision_Detector::found_collisions() const
{
    L_ASSERT(m_broad_phase && m_narrow_phase);
    return m_narrow_phase->get_collisions();
}
