#include <Modules/Physics_Module.h>

using namespace LPhys;


void Physics_Module::expand_border(Border& _border) const
{

}

bool Physics_Module::may_intersect_with_other(const Physics_Module& _other) const
{
    return true;
}

bool Physics_Module::intersects_with_border(const Border& _border) const
{
    return true;
}



LEti::Transformation_Data Physics_Module::calculate_transformation_data_after_collision() const
{
    LEti::Transformation_Data result = *transformation_data();

    if(m_transformations_after_collisions.size() == 0)
        return result;

    glm::vec3 average_rotation(0.0f, 0.0f, 0.0f);
    glm::vec3 average_scale(0.0f, 0.0f, 0.0f);

    glm::vec3 largest_movement_vec(0.0f, 0.0f, 0.0f);

    for(Transformations_List::Const_Iterator it = m_transformations_after_collisions.begin(); !it.end_reached(); ++it)
    {
        const LEti::Transformation_Data& transformation = *it;

        glm::vec3 movement_vec = transformation.position() - result.position();

        for(unsigned int i = 0; i < 3; ++i)
        {
            if(fabsf(largest_movement_vec[i]) >= fabsf(movement_vec[i]))
                continue;

            largest_movement_vec[i] = movement_vec[i];
        }

        average_rotation += LST::Math::calculate_angles(transformation.rotation());
        average_scale += transformation.scale();
    }

    float multiplier = 1.0f / (float)(m_transformations_after_collisions.size());

    average_rotation *= multiplier;
    average_scale *= multiplier;

    result.move(largest_movement_vec);
    result.set_rotation(average_rotation);
    result.set_scale(average_scale);

    return result;
}

void Physics_Module::apply_data_after_collisions()
{
    *transformation_data() = calculate_transformation_data_after_collision();

    m_transformations_after_collisions.clear();

    update(0.0f);
}





BUILDER_STUB_NULL_CONSTRUCTION_FUNC(Physics_Module_Stub)

BUILDER_STUB_INITIALIZATION_FUNC(Physics_Module_Stub)
{
    BUILDER_STUB_PARENT_INITIALIZATION;
    BUILDER_STUB_CAST_PRODUCT;

    if(!allow_collisions)
        product->allow_collisions(false);

    product->set_static(is_static);
}
