#pragma once

#include <Stuff/Function_Wrapper.h>

#include <Modules/Physics_Module_2D.h>
#include <Physical_Models/Rigid_Body_Physical_Model_2D.h>


namespace LPhys
{

    class Rigid_Body_2D : public Physics_Module_2D
	{
    public:
        INIT_VARIABLE(LPhys::Rigid_Body_2D, LPhys::Physics_Module_2D)

	private:
        float m_mass_multiplier = 1.0f;
        glm::vec3 m_velocity{0.0f, 0.0f, 0.0f};
        float m_angular_velocity = 0.0f;

        LST::Function<void()> m_on_alignment;

    private:
        Physical_Model_2D* M_create_physical_model() const override;

    public:
        glm::vec3 calculate_raw_center_of_mass() const;

    public:
        void align_to_center_of_mass();

    public:
        void set_masses(const float* _masses);

        inline void set_mass_multiplier(float _mass_multiplier) { m_mass_multiplier = _mass_multiplier; }
        inline void set_velocity(const glm::vec3& _v) { m_velocity = _v; }
        inline void set_angular_velocity(float _av) { m_angular_velocity = _av; }

        inline void apply_linear_impulse(const glm::vec3& _imp) { m_velocity += _imp; }
        inline void apply_rotation(float _av) { m_angular_velocity += _av; }

        inline void set_on_alignment_func(const LST::Function<void()>& _on_alignment) { m_on_alignment = _on_alignment; }

    public:
        float mass() const;
        float moment_of_inertia() const;

        inline float mass_multiplier() const { return m_mass_multiplier; }
        inline const glm::vec3& velocity() const { return m_velocity; }
        inline float angular_velocity() const { return m_angular_velocity; }

    public:
        void update(float _dt) override;

	};



    class Rigid_Body_2D__Stub : public Physics_Module_2D_Stub
    {
    public:
        INIT_VARIABLE(LPhys::Rigid_Body_2D__Stub, LPhys::Physics_Module_2D_Stub)

        INIT_FIELDS
        ADD_FIELD(LDS::Vector<float>, masses)
        ADD_FIELD(float, mass_multiplier)
        FIELDS_END

    public:
        LDS::Vector<float> masses;
        float mass_multiplier = 1.0f;

    protected:
        INIT_BUILDER_STUB(Rigid_Body_2D)
    };

}
