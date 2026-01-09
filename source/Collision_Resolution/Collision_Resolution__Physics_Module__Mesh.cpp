#include <Collision_Resolution/Collision_Resolution__Physics_Module__Mesh.h>
#include <Modules/Physics_Module__Mesh.h>

using namespace LPhys;


bool Collision_Resolution__Physics_Module__Mesh::M_resolve_dynamic_vs_dynamic(const Intersection_Data &_id, float _dt)
{
    L_ASSERT(LV::cast_variable<Physics_Module__Mesh>(_id.first));
    L_ASSERT(LV::cast_variable<Physics_Module__Mesh>(_id.second));

    Physics_Module__Mesh* pm1 = (Physics_Module__Mesh*)_id.first;
    Physics_Module__Mesh* pm2 = (Physics_Module__Mesh*)_id.second;

    LEti::Transformation_Data transformation_data_after_collision_1 = LEti::Transformation_Data::get_transformation_data_for_ratio(*pm1->transformation_data_prev_state(), *pm1->transformation_data(), _id.time_of_intersection_ratio);
    LEti::Transformation_Data transformation_data_after_collision_2 = LEti::Transformation_Data::get_transformation_data_for_ratio(*pm2->transformation_data_prev_state(), *pm2->transformation_data(), _id.time_of_intersection_ratio);

    glm::vec3 separation_vec = _id.normal * _id.depth * 1.01f;

    transformation_data_after_collision_1.move(  (separation_vec * 0.5f) );
    transformation_data_after_collision_2.move( -(separation_vec * 0.5f) );

    float time_after_intersection_ratio = 1.0f - _id.time_of_intersection_ratio;

    {
        glm::vec3 stride_vec = pm1->transformation_data()->position() - pm1->transformation_data_prev_state()->position();
        stride_vec *= time_after_intersection_ratio;
        glm::vec3 stride_on_normal_projection = LST::Math::dot_product(stride_vec, _id.normal) * _id.normal;
        stride_vec -= stride_on_normal_projection;

        transformation_data_after_collision_1.move(stride_vec);
    }
    {
        glm::vec3 stride_vec = pm2->transformation_data()->position() - pm2->transformation_data_prev_state()->position();
        stride_vec *= time_after_intersection_ratio;
        glm::vec3 stride_on_normal_projection = LST::Math::dot_product(stride_vec, -_id.normal) * -_id.normal;
        stride_vec -= stride_on_normal_projection;

        transformation_data_after_collision_2.move(stride_vec);
    }

    pm1->add_transformation_after_collision(transformation_data_after_collision_1);
    pm2->add_transformation_after_collision(transformation_data_after_collision_2);

    return true;
}

bool Collision_Resolution__Physics_Module__Mesh::M_resolve_dynamic_vs_static(const Intersection_Data &_id, float _dt)
{
    L_ASSERT(LV::cast_variable<Physics_Module__Mesh>(_id.first));
    L_ASSERT(LV::cast_variable<Physics_Module__Mesh>(_id.second));

    Physics_Module__Mesh* pm1 = (Physics_Module__Mesh*)_id.first;
    Physics_Module__Mesh* pm2 = (Physics_Module__Mesh*)_id.second;

    glm::vec3 normal = _id.normal;

    if(pm1->is_static())
    {
        Physics_Module__Mesh* temp = pm1;
        pm1 = pm2;
        pm2 = temp;

        normal *= -1.0f;
    }

    LEti::Transformation_Data transformation_data_after_collision_1 = LEti::Transformation_Data::get_transformation_data_for_ratio(*pm1->transformation_data_prev_state(), *pm1->transformation_data(), _id.time_of_intersection_ratio);

    glm::vec3 separation_vec = normal * _id.depth * 1.01f;

    transformation_data_after_collision_1.move(separation_vec);

    float time_after_intersection_ratio = 1.0f - _id.time_of_intersection_ratio;

    glm::vec3 stride_vec = pm1->transformation_data()->position() - pm1->transformation_data_prev_state()->position();
    stride_vec *= time_after_intersection_ratio;
    glm::vec3 stride_on_normal_projection = LST::Math::dot_product(stride_vec, normal) * normal;
    stride_vec -= stride_on_normal_projection;

    transformation_data_after_collision_1.move(stride_vec);

    pm1->add_transformation_after_collision(transformation_data_after_collision_1);

    return true;
}



bool Collision_Resolution__Physics_Module__Mesh::resolve(const Intersection_Data& _id, float _dt)
{
    if(!_id.first->is_static() && !_id.second->is_static())
        return M_resolve_dynamic_vs_dynamic(_id, _dt);

    if(_id.first->is_static() && _id.second->is_static())
        return false;

    return M_resolve_dynamic_vs_static(_id, _dt);
}
