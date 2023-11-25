#include <Collision_Resolution/Collision_Resolution__Rigid_Body_2D.h>

using namespace LPhys;


float Collision_Resolution__Rigid_Body_2D::M_calculate_kinetic_energy(const glm::vec3& _velocity, float _angular_velocity, float _mass, float _moment_of_inertia) const
{
    float velocity = LEti::Math::vector_length(_velocity);

    float movemental = (_mass * velocity * velocity) / 2.0f;
    float rotational = (_moment_of_inertia * _angular_velocity * _angular_velocity) / 2.0f;

    return movemental + rotational;
}



bool Collision_Resolution__Rigid_Body_2D::resolve(const Intersection_Data &_id, float _dt)
{
    Rigid_Body_2D* pm1 = LV::cast_variable<Rigid_Body_2D>((Physics_Module_2D*)_id.first);
    Rigid_Body_2D* pm2 = LV::cast_variable<Rigid_Body_2D>((Physics_Module_2D*)_id.second);

    if(!pm1 || !pm2)
        return false;

    glm::vec3 A_center_of_mas = pm1->get_physical_model()->center_of_mass();
    glm::vec3 B_center_of_mas = pm2->get_physical_model()->center_of_mass();

    float A_moment_of_inertia = pm1->moment_of_inertia();
    float B_moment_of_inertia = pm2->moment_of_inertia();

    glm::vec3 A_velocity = pm1->velocity();
    glm::vec3 B_velocity = pm2->velocity();
    float A_angular_velocity = pm1->angular_velocity();
    float B_angular_velocity = pm2->angular_velocity();

    float ke_before = M_calculate_kinetic_energy(A_velocity, A_angular_velocity, pm1->mass(), A_moment_of_inertia) + M_calculate_kinetic_energy(B_velocity, B_angular_velocity, pm2->mass(), B_moment_of_inertia);

    pm1->transformation_data()->set_position( LEti::Transformation_Data::get_position_for_ratio(*pm1->transformation_data_prev_state(), *pm1->transformation_data(), _id.time_of_intersection_ratio) );
    pm2->transformation_data()->set_position( LEti::Transformation_Data::get_position_for_ratio(*pm2->transformation_data_prev_state(), *pm2->transformation_data(), _id.time_of_intersection_ratio) );

    float e = 1.0f;

    glm::vec3 ra = _id.point - A_center_of_mas;
    glm::vec3 rb = _id.point - B_center_of_mas;

    glm::vec3 raPerp = {-ra.y, ra.x, 0.0f};
    glm::vec3 rbPerp = {-rb.y, rb.x, 0.0f};

    //	angular linear velocity
    glm::vec3 alvA = raPerp * A_angular_velocity;
    glm::vec3 alvB = rbPerp * B_angular_velocity;

    glm::vec3 relativeVelocity = (B_velocity + alvB) - (A_velocity + alvA);

    float contactVelocityMag = LEti::Math::dot_product(relativeVelocity, _id.normal);

    float raPerpDotN = LEti::Math::dot_product(raPerp, _id.normal);
    float rbPerpDotN = LEti::Math::dot_product(rbPerp, _id.normal);

    float denom = 1 / pm1->mass() + 1 / pm2->mass() +
            (raPerpDotN * raPerpDotN) / A_moment_of_inertia +
            (rbPerpDotN * rbPerpDotN) / B_moment_of_inertia;

    float j = -(1.0f + e) * contactVelocityMag;
    j /= denom;

    glm::vec3 impulse = j * _id.normal;

    float avA = LEti::Math::cross_product(ra, impulse).z / A_moment_of_inertia;
    float avB = LEti::Math::cross_product(rb, impulse).z / B_moment_of_inertia;

    Rigid_Body_2D* heavier_pm = pm1->mass() > pm2->mass() ? pm1 : pm2;
    Rigid_Body_2D* lighter_pm = heavier_pm == pm1 ? pm2 : pm1;

    float masses_ratio = lighter_pm->mass() / heavier_pm->mass();

    glm::vec3 separation_vec = _id.normal * (_id.depth * 1.01f);
    glm::vec3 heavier_pm_separation_vec = separation_vec * masses_ratio;
    glm::vec3 lighter_pm_separation_vec = separation_vec * (1.0f - masses_ratio);

    pm1->transformation_data()->move(  (pm1 == heavier_pm ? heavier_pm_separation_vec : lighter_pm_separation_vec) );
    pm2->transformation_data()->move( -(pm2 == heavier_pm ? heavier_pm_separation_vec : lighter_pm_separation_vec) );

//    pm1->transformation_data()->move(separation_vec * (1.0f / masses_ratio));
//    pm2->transformation_data()->move(-separation_vec * masses_ratio);

    pm1->apply_linear_impulse(-impulse / pm1->mass());
    pm1->apply_rotation(-avA);
    pm2->apply_linear_impulse(impulse / pm2->mass());
    pm2->apply_rotation(avB);

    //  attempt to fix increase of models' velocities after some collisions. and it seems to work fine!
    float ke_after = M_calculate_kinetic_energy(pm1->velocity(), pm1->angular_velocity(), pm1->mass(), A_moment_of_inertia) + M_calculate_kinetic_energy(pm2->velocity(), pm2->angular_velocity(), pm2->mass(), B_moment_of_inertia);

    if(!LEti::Math::floats_are_equal(ke_after, 0.0f) && !LEti::Math::floats_are_equal(ke_before, 0.0f))
    {
        float ratio_sqrt = sqrtf(ke_before / ke_after);

        pm1->set_velocity(pm1->velocity() * ratio_sqrt);
        pm1->set_angular_velocity(pm1->angular_velocity() * ratio_sqrt);
        pm2->set_velocity(pm2->velocity() * ratio_sqrt);
        pm2->set_angular_velocity(pm2->angular_velocity() * ratio_sqrt);
    }

    pm1->update(_dt * (1.0f - _id.time_of_intersection_ratio));
    pm2->update(_dt * (1.0f - _id.time_of_intersection_ratio));

    return true;
}
