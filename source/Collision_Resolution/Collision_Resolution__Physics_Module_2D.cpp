#include <Collision_Resolution/Collision_Resolution__Physics_Module_2D.h>

using namespace LPhys;


void Collision_Resolution__Physics_Module_2D::resolve(const Intersection_Data &_id, float _dt)
{
    L_ASSERT(LV::cast_variable<Physics_Module_2D>(_id.first));
    L_ASSERT(LV::cast_variable<Physics_Module_2D>(_id.second));

    Physics_Module_2D* pm1 = (Physics_Module_2D*)_id.first;
    Physics_Module_2D* pm2 = (Physics_Module_2D*)_id.second;

    glm::vec3 separation_vec = _id.normal * _id.depth * 1.01f;

    pm1->transformation_data()->move(  (separation_vec * 0.5f) );
    pm2->transformation_data()->move( -(separation_vec * 0.5f) );
}
