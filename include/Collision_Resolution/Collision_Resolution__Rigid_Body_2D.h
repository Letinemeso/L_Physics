#pragma once

#include <Collision_Resolution/Collision_Resolver.h>
#include <Modules/Rigid_Body_2D.h>


namespace LPhys
{

	class Collision_Resolution__Rigid_Body_2D : public Collision_Resolution_Interface
	{
    private:
        float M_calculate_kinetic_energy(const glm::vec3& _velocity, float _angular_velocity, float _mass, float _moment_of_inertia) const;

	public:
        void resolve(const Intersection_Data &_id, float _dt = 0.0f) override;

	};

}
