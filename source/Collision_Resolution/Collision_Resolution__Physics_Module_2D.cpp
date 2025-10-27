#include <Collision_Resolution/Collision_Resolution__Physics_Module_2D.h>
#include <Modules/Physics_Module__Mesh.h>

using namespace LPhys;


bool Collision_Resolution__Physics_Module_2D::resolve(const Intersection_Data &_id, float _dt)
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
        glm::vec3 stride_on_normal_projection = LEti::Math::dot_product(stride_vec, _id.normal) * _id.normal;
        stride_vec -= stride_on_normal_projection;

        transformation_data_after_collision_1.move(stride_vec);
    }
    {
        glm::vec3 stride_vec = pm2->transformation_data()->position() - pm2->transformation_data_prev_state()->position();
        stride_vec *= time_after_intersection_ratio;
        glm::vec3 stride_on_normal_projection = LEti::Math::dot_product(stride_vec, -_id.normal) * -_id.normal;
        stride_vec -= stride_on_normal_projection;

        transformation_data_after_collision_2.move(stride_vec);
    }

    pm1->add_transformation_after_collision(transformation_data_after_collision_1);
    pm2->add_transformation_after_collision(transformation_data_after_collision_2);

    return true;
}
