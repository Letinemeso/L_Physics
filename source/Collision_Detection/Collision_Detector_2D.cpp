#include <Collision_Detection/Collision_Detector_2D.h>

using namespace LPhys;


Collision_Detector_2D::Collision_Detector_2D()
{

}

Collision_Detector_2D::~Collision_Detector_2D()
{
	delete m_broad_phase;
	delete m_narrow_phase;
    delete m_narrowest_phase;
}



void Collision_Detector_2D::debug_assert_if_model_copy_found(const Physics_Module *_model, bool _reverse)
{
    Registred_Modules_List::Iterator check = m_registred_modules.begin();
    while(!check.end_reached())
	{
        L_ASSERT(!_reverse && *check != _model);
		++check;
	}

	L_ASSERT(!_reverse);
}



void Collision_Detector_2D::set_broad_phase(Broad_Phase_Interface* _broad_phase_impl)
{
	delete m_broad_phase;
    m_broad_phase = _broad_phase_impl;
}

void Collision_Detector_2D::set_narrow_phase(Narrow_Phase_Interface* _narrow_phase_impl)
{
	delete m_narrow_phase;
    m_narrow_phase = _narrow_phase_impl;
}

void Collision_Detector_2D::set_narrowest_phase(Narrowest_Phase_Interface* _narrowest_phase_impl)
{
    delete m_narrowest_phase;
    m_narrowest_phase = _narrowest_phase_impl;
}



void Collision_Detector_2D::register_object(const Physics_Module *_model)
{
	L_DEBUG_FUNC_2ARG(debug_assert_if_model_copy_found, _model, false);
    m_registred_modules.push_back(_model);
}

void Collision_Detector_2D::unregister_object(const Physics_Module *_model)
{
    Registred_Modules_List::Iterator it = m_registred_modules.begin();
    while(!it.end_reached())
	{
        if(*it == _model)
            break;
		++it;
	}
    L_ASSERT(!it.end_reached());
    m_registred_modules.erase(it);
}

void Collision_Detector_2D::unregister_all_objects()
{
    m_registred_modules.clear();
}



void Collision_Detector_2D::update()
{
    L_ASSERT(m_broad_phase && m_narrow_phase && m_narrowest_phase);

    m_broad_phase->update(m_registred_modules);
    m_narrow_phase->update(m_broad_phase->possible_collisions__models(), m_narrowest_phase);
}



const Narrow_Phase_Interface::Collision_Data_List__Models& Collision_Detector_2D::get_collisions__models()
{
    L_ASSERT(m_broad_phase && m_narrow_phase && m_narrowest_phase);
	return m_narrow_phase->get_collisions__models();
}
