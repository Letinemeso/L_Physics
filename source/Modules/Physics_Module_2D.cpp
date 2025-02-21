#include <Modules/Physics_Module_2D.h>

using namespace LPhys;


Physics_Module_2D::Physics_Module_2D()
{

}

Physics_Module_2D::~Physics_Module_2D()
{
	delete m_physical_model_prev_state;
	delete m_physical_model;
}



Physical_Model_2D* Physics_Module_2D::M_create_physical_model() const
{
    return new Physical_Model_2D;
}



void Physics_Module_2D::M_can_collide_changed()
{
    if(!can_collide())
        return;

    update(0.0f);
    update_prev_state();
}



void Physics_Module_2D::init_physical_model()
{
	delete m_physical_model;
    m_physical_model = nullptr;

    m_physical_model = M_create_physical_model();
}

void Physics_Module_2D::init_prev_state()
{
    delete m_physical_model_prev_state;
    m_physical_model_prev_state = m_physical_model->create_imprint();
}

void Physics_Module_2D::setup_base_data(const float* _raw_coords, unsigned int _raw_coords_count, const bool* _collision_permissions)
{
    m_physical_model->setup(_raw_coords, _raw_coords_count, _collision_permissions);

    if(!m_physical_model_prev_state)
        init_prev_state();
}


void Physics_Module_2D::move_raw(const glm::vec3 &_stride)
{
    m_physical_model->move_raw(_stride);
}



void Physics_Module_2D::update_prev_state()
{
    if(!can_collide())
        return;

	L_ASSERT(!(!m_physical_model || !m_physical_model_prev_state));

	m_physical_model_prev_state->update_to_current_model_state();
}

void Physics_Module_2D::update(float /*_dt*/)
{
    if(!can_collide())
        return;

    L_ASSERT(m_physical_model && m_physical_model_prev_state && transformation_data());

    transformation_data()->update_matrix();

    m_physical_model->update(transformation_data()->matrix());

    m_border = get_physical_model_prev_state()->border() || get_physical_model()->border();
}



void Physics_Module_2D::expand_border(Border& _border) const
{
    _border = _border || m_border;
}

bool Physics_Module_2D::may_intersect_with_other(const Physics_Module& _other) const
{
    return _other.intersects_with_border(m_border);
}

bool Physics_Module_2D::intersects_with_border(const Border& _border) const
{
    return m_border && _border;
}





LV::Variable_Base* Physics_Module_2D_Stub::M_construct_product() const
{
    return new Physics_Module_2D;
}

void Physics_Module_2D_Stub::M_init_constructed_product(LV::Variable_Base* _product) const
{
    Physics_Module_2D* result = (Physics_Module_2D*)_product;

    result->init_physical_model();

    if(coords.size() > 0)
        result->setup_base_data(coords.raw_data(), coords.size(), collision_permissions.raw_data());

    if(on_collision_func)
        result->set_on_collision_function(on_collision_func);
}
