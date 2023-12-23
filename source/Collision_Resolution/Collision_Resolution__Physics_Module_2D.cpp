#include <Collision_Resolution/Collision_Resolution__Physics_Module_2D.h>

using namespace LPhys;


bool Collision_Resolution__Physics_Module_2D::resolve(const Intersection_Data &_id, float _dt)
{
    Physics_Module_2D* pm1 = (Physics_Module_2D*)_id.first;
    Physics_Module_2D* pm2 = (Physics_Module_2D*)_id.second;

    if(!pm1 || !pm2)
        return false;

    //  this stuff completely ignores potential rotation, but i don't need it right now, so let it be future problem

    float movement_magnitude_1 = LEti::Math::vector_length(pm1->transformation_data_prev_state()->position() - pm1->transformation_data()->position());
    float movement_magnitude_2 = LEti::Math::vector_length(pm2->transformation_data_prev_state()->position() - pm2->transformation_data()->position());
    float total_movement_magnitude = movement_magnitude_1 + movement_magnitude_2;

    if(total_movement_magnitude < 0.0001f)
        return true;

    movement_magnitude_1 /= total_movement_magnitude;
    movement_magnitude_2 /= total_movement_magnitude;

    glm::vec3 separation_vec = _id.normal * _id.depth * 1.01f;

    pm1->transformation_data()->move(  (separation_vec * movement_magnitude_1) );
    pm2->transformation_data()->move( -(separation_vec * movement_magnitude_2) );

    return true;
}
