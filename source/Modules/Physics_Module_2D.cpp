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



void Physics_Module_2D::init_physical_model()
{
	delete m_physical_model;
    m_physical_model = nullptr;

    m_physical_model = M_create_physical_model();
}

void Physics_Module_2D::init_prev_state()
{
    delete m_physical_model_prev_state;
    m_physical_model_prev_state = nullptr;

    m_physical_model_prev_state = m_physical_model->create_imprint();
}

void Physics_Module_2D::setup_base_data(const float* _raw_coords, unsigned int _raw_coords_count, const bool* _collision_permissions)
{
    m_physical_model->setup(_raw_coords, _raw_coords_count, _collision_permissions);
}


void Physics_Module_2D::move_raw(const glm::vec3 &_stride)
{
    m_physical_model->move_raw(_stride);
}



void Physics_Module_2D::update_prev_state()
{
	L_ASSERT(!(!m_physical_model || !m_physical_model_prev_state));

	m_physical_model_prev_state->update_to_current_model_state();
}

void Physics_Module_2D::update(float /*_dt*/)
{
    L_ASSERT(m_physical_model && m_physical_model_prev_state && transformation_data());

    transformation_data()->update_matrix();

    m_physical_model->update(transformation_data()->matrix());

        const LEti::Geometry_2D::Rectangular_Border& prev_rb = get_physical_model_prev_state()->curr_rect_border(),
			curr_rb = get_physical_model()->curr_rect_border();

    m_rectangular_border.left = prev_rb.left < curr_rb.left ? prev_rb.left : curr_rb.left;
    m_rectangular_border.right = prev_rb.right > curr_rb.right ? prev_rb.right : curr_rb.right;
    m_rectangular_border.top = prev_rb.top > curr_rb.top ? prev_rb.top : curr_rb.top;
    m_rectangular_border.bottom = prev_rb.bottom < curr_rb.bottom ? prev_rb.bottom : curr_rb.bottom;
}





LV::Variable_Base* Physics_Module_2D_Stub::M_construct_product() const
{
    return new Physics_Module_2D;
}

void Physics_Module_2D_Stub::M_init_constructed_product(LV::Variable_Base* _product) const
{
    Physics_Module_2D* result = (Physics_Module_2D*)_product;

    result->init_physical_model();
    result->setup_base_data(coords, coords_count, collision_permissions);
    result->init_prev_state();
}



Physics_Module_2D_Stub::~Physics_Module_2D_Stub()
{
    delete[] coords;
    delete[] collision_permissions;
}
