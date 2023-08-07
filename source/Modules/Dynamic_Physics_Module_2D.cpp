#include <Modules/Dynamic_Physics_Module_2D.h>

using namespace LPhys;


INIT_FIELDS(LPhys::Dynamic_Physics_Module_2D_Stub, LPhys::Physics_Module_Base_Stub)

ADD_FIELD(unsigned int, coords_count)
ADD_FIELD(float*, coords)

ADD_FIELD(bool*, collision_permissions)

FIELDS_END



LV::Variable_Base* Dynamic_Physics_Module_2D_Stub::M_construct_product() const
{
    return new Dynamic_Physics_Module_2D;
}

void Dynamic_Physics_Module_2D_Stub::M_init_constructed_product(LV::Variable_Base* _product) const
{
    Dynamic_Physics_Module_2D* result = (Dynamic_Physics_Module_2D*)_product;

    result->init_physical_model();
    result->setup_base_data(coords, coords_count, collision_permissions);
    result->init_prev_state();
}



Dynamic_Physics_Module_2D_Stub::~Dynamic_Physics_Module_2D_Stub()
{
    delete[] coords;
    delete[] collision_permissions;
}


INIT_FIELDS(LPhys::Dynamic_Physics_Module_2D, LPhys::Physics_Module_Base)
FIELDS_END



Dynamic_Physics_Module_2D::Dynamic_Physics_Module_2D()
{

}

Dynamic_Physics_Module_2D::~Dynamic_Physics_Module_2D()
{
	delete m_physical_model_prev_state;
	delete m_physical_model;
}



Physical_Model_2D* Dynamic_Physics_Module_2D::M_create_physical_model() const
{
    return new Physical_Model_2D;
}



void Dynamic_Physics_Module_2D::init_physical_model()
{
	delete m_physical_model;
    m_physical_model = nullptr;

    m_physical_model = M_create_physical_model();
}

void Dynamic_Physics_Module_2D::init_prev_state()
{
    delete m_physical_model_prev_state;
    m_physical_model_prev_state = nullptr;

    m_physical_model_prev_state = m_physical_model->create_imprint();
}

void Dynamic_Physics_Module_2D::setup_base_data(const float* _raw_coords, unsigned int _raw_coords_count, const bool* _collision_permissions)
{
    m_physical_model->setup(_raw_coords, _raw_coords_count, _collision_permissions);
}


void Dynamic_Physics_Module_2D::move_raw(const glm::vec3 &_stride)
{
    m_physical_model->move_raw(_stride);
}



void Dynamic_Physics_Module_2D::update_previous_state()
{
	L_ASSERT(!(!m_physical_model || !m_physical_model_prev_state));

    if(!can_collide())
        return;

	m_physical_model_prev_state->update_to_current_model_state();
}

void Dynamic_Physics_Module_2D::update(const glm::mat4x4 &_matrix)
{
	L_ASSERT(!(!m_physical_model || !m_physical_model_prev_state));

    if(!can_collide())
        return;

    m_physical_model->update(_matrix);

        const LEti::Geometry_2D::Rectangular_Border& prev_rb = get_physical_model_prev_state()->curr_rect_border(),
			curr_rb = get_physical_model()->curr_rect_border();

    m_rectangular_border.left = prev_rb.left < curr_rb.left ? prev_rb.left : curr_rb.left;
    m_rectangular_border.right = prev_rb.right > curr_rb.right ? prev_rb.right : curr_rb.right;
    m_rectangular_border.top = prev_rb.top > curr_rb.top ? prev_rb.top : curr_rb.top;
    m_rectangular_border.bottom = prev_rb.bottom < curr_rb.bottom ? prev_rb.bottom : curr_rb.bottom;
}
