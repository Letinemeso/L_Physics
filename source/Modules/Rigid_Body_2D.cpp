#include <Modules/Rigid_Body_2D.h>

using namespace LPhys;


Physical_Model_2D* Rigid_Body_2D::M_create_physical_model() const
{
    return new Rigid_Body_Physical_Model_2D;
}



glm::vec3 Rigid_Body_2D::calculate_raw_center_of_mass() const
{
    glm::vec3 result(0.0f, 0.0f, 0.0f);

    for(unsigned int i=0; i<get_physical_model()->get_polygons_count(); ++i)
    {
        const Rigid_Body_Polygon& polygon = (const Rigid_Body_Polygon&)*get_physical_model()->get_polygon(i);
        result += polygon.center_raw() * polygon.mass();
    }

    result /= ((Rigid_Body_Physical_Model_2D*)get_physical_model())->total_mass();

    return result;
}



void Rigid_Body_2D::align_to_center_of_mass()
{
    if(m_on_alignment)
        m_on_alignment();

	glm::vec3 stride = -calculate_raw_center_of_mass();

	get_physical_model()->move_raw(stride);
	get_physical_model_prev_state()->update_to_current_model_state();
}



void Rigid_Body_2D::set_masses(const float* _masses)
{
    Rigid_Body_Physical_Model_2D* physical_model = (Rigid_Body_Physical_Model_2D*)get_physical_model();

    physical_model->set_masses(_masses);
}



float Rigid_Body_2D::mass() const
{
    Rigid_Body_Physical_Model_2D* physical_model = (Rigid_Body_Physical_Model_2D*)get_physical_model();

    return physical_model->total_mass() * m_mass_multiplier;
}

float Rigid_Body_2D::moment_of_inertia() const
{
    Rigid_Body_Physical_Model_2D* physical_model = (Rigid_Body_Physical_Model_2D*)get_physical_model();

    return physical_model->moment_of_inertia() * m_mass_multiplier;
}



void Rigid_Body_2D::update(float _dt)
{
    L_ASSERT(get_physical_model() && get_physical_model_prev_state() && transformation_data());

    transformation_data()->move(velocity() * _dt);
    transformation_data()->rotate({0.0f, 0.0f, angular_velocity() * _dt});

    Physics_Module_2D::update(_dt);
}





BUILDER_STUB_DEFAULT_CONSTRUCTION_FUNC(Rigid_Body_2D__Stub)

BUILDER_STUB_INITIALIZATION_FUNC(Rigid_Body_2D__Stub)
{
    BUILDER_STUB_PARENT_INITIALIZATION;
    BUILDER_STUB_CAST_PRODUCT;

    product->init_physical_model();
    product->setup_base_data(coords.raw_data(), coords.size(), collision_permissions.raw_data());
    product->set_masses(masses.raw_data());
    product->set_mass_multiplier(mass_multiplier);
    product->init_prev_state();
}
