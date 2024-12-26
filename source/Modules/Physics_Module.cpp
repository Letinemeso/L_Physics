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



void Physics_Module::apply_data_after_collisions()
{
    if(m_transformations_after_collisions.size() == 0)
        return;

    glm::vec3 average_position(0.0f, 0.0f, 0.0f);
    glm::vec3 average_rotation(0.0f, 0.0f, 0.0f);
    glm::vec3 average_scale(0.0f, 0.0f, 0.0f);

    for(Transformations_List::Iterator it = m_transformations_after_collisions.begin(); !it.end_reached(); ++it)
    {
        const LEti::Transformation_Data& transformation = *it;

        average_position += transformation.position();
        average_rotation += transformation.rotation();
        average_scale += transformation.scale();
    }

    float multiplier = 1.0f / (float)(m_transformations_after_collisions.size());

    average_position *= multiplier;
    average_rotation *= multiplier;
    average_scale *= multiplier;

    transformation_data()->set_position(average_position);
    transformation_data()->set_rotation(average_rotation);
    transformation_data()->set_scale(average_scale);

    m_transformations_after_collisions.clear();
}
