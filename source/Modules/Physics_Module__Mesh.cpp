#include <Modules/Physics_Module__Mesh.h>

using namespace LPhys;


Physics_Module__Mesh::Physics_Module__Mesh()
{

}

Physics_Module__Mesh::~Physics_Module__Mesh()
{
	delete m_physical_model_prev_state;
	delete m_physical_model;
}



Physical_Model* Physics_Module__Mesh::M_create_physical_model() const
{
    return new Physical_Model;
}



void Physics_Module__Mesh::M_can_collide_changed()
{
    if(!can_collide())
        return;

    update(0.0f);
    update_prev_state();
}

void Physics_Module__Mesh::M_on_parent_object_set()
{
    update_physical_model();
}



void Physics_Module__Mesh::setup_base_data(const float* _raw_coords, unsigned int _raw_coords_count, const bool* _collision_permissions)
{
    delete m_physical_model;
    delete m_physical_model_prev_state;

    m_physical_model = M_create_physical_model();

    m_physical_model->setup(_raw_coords, _raw_coords_count, _collision_permissions);

    if(transformation_data())
        m_physical_model->update(transformation_data()->matrix());

    m_physical_model_prev_state = m_physical_model->create_imprint();
}


void Physics_Module__Mesh::move_raw(const glm::vec3 &_stride)
{
    L_ASSERT(m_physical_model);
    m_physical_model->move_raw(_stride);
}



void Physics_Module__Mesh::update_prev_state()
{
    if(!can_collide())
        return;

    L_ASSERT(m_physical_model && m_physical_model_prev_state);

	m_physical_model_prev_state->update_to_current_model_state();
}

void Physics_Module__Mesh::update(float /*_dt*/)
{
    if(!can_collide())
        return;

    L_ASSERT(m_physical_model && m_physical_model_prev_state && transformation_data());

    if(transformation_data()->modified())
        m_physical_model->update(transformation_data()->matrix());

    m_border = get_physical_model_prev_state()->border() || get_physical_model()->border();
}

void Physics_Module__Mesh::update_physical_model()
{
    if(!m_physical_model)
        return;
    if(m_physical_model->get_polygons()->amount() == 0)
        return;

    m_physical_model->update(transformation_data()->matrix());
    m_physical_model_prev_state->update_to_current_model_state();
    m_border = get_physical_model_prev_state()->border() || get_physical_model()->border();
}



void Physics_Module__Mesh::expand_border(Border& _border) const
{
    _border.expand_with(m_border);
}

bool Physics_Module__Mesh::may_intersect_with_other(const Physics_Module& _other) const
{
    return _other.intersects_with_border(m_border);
}

bool Physics_Module__Mesh::intersects_with_border(const Border& _border) const
{
    return m_border.intersects_with(_border);
}





Physics_Module_Stub__Mesh::~Physics_Module_Stub__Mesh()
{
    delete data_provider;
}



const LDS::Vector<float>& Physics_Module_Stub__Mesh::M_select_data() const
{
    if(coords.size() > 0)
        return coords;

    if(!data_provider)
        return coords;

    return data_provider->get_data();
}



BUILDER_STUB_DEFAULT_CONSTRUCTION_FUNC(Physics_Module_Stub__Mesh)

BUILDER_STUB_INITIALIZATION_FUNC(Physics_Module_Stub__Mesh)
{
    BUILDER_STUB_PARENT_INITIALIZATION;
    BUILDER_STUB_CAST_PRODUCT;

    const LDS::Vector<float>& data = M_select_data();

    if(data.size() > 0)
    {
        const bool* raw_collisions_permissions = nullptr;
        if(collision_permissions.size() > 0)
            raw_collisions_permissions = collision_permissions.raw_data();

        product->setup_base_data(data.raw_data(), data.size(), raw_collisions_permissions);
    }

    if(on_collision_func)
        product->set_on_collision_function(on_collision_func);
}
