#pragma once

#include <Stuff/Function_Wrapper.h>

#include <Builder_Stub.h>

#include <Modules/Physics_Module.h>
#include <Physical_Models/Physical_Model_2D.h>


namespace LPhys
{

    class Physics_Module_2D : public Physics_Module
	{
    public:
        INIT_VARIABLE(LPhys::Physics_Module_2D, LPhys::Physics_Module);

    protected:
        Border m_border;

    private:
        Physical_Model_2D* m_physical_model = nullptr;
        Physical_Model_2D_Imprint* m_physical_model_prev_state = nullptr;

	public:
        Physics_Module_2D();
        ~Physics_Module_2D();

    private:
        virtual Physical_Model_2D* M_create_physical_model() const;

    protected:
        void M_can_collide_changed() override;
        void M_on_parent_object_set() override;

    public:
        void setup_base_data(const float* _raw_coords, unsigned int _raw_coords_count, const bool* _collision_permissions);

        void move_raw(const glm::vec3 &_stride);

    public:
        void update_prev_state() override;
        void update(float _dt) override;
        void update_physical_model();

    public:
        void expand_border(Border& _border) const override;
        bool may_intersect_with_other(const Physics_Module& _other) const override;
        bool intersects_with_border(const Border& _border) const override;

    public:
        inline const Border& border() const { return m_border; }
        inline Physical_Model_2D* get_physical_model() { return m_physical_model; }
        inline Physical_Model_2D_Imprint* get_physical_model_prev_state() { return m_physical_model_prev_state; }
        inline const Physical_Model_2D* get_physical_model() const { return m_physical_model; }
        inline const Physical_Model_2D_Imprint* get_physical_model_prev_state() const { return m_physical_model_prev_state; }

	};



    class Physics_Module_2D_Stub : public Physics_Module_Stub
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module_2D_Stub, LPhys::Physics_Module_Stub)

        INIT_FIELDS
        ADD_FIELD(LDS::Vector<float>, coords)
        ADD_FIELD(LDS::Vector<bool>, collision_permissions)
        FIELDS_END

    public:
        LDS::Vector<float> coords;
        LDS::Vector<bool> collision_permissions;

    public:
        Physics_Module_2D::On_Collision_Function on_collision_func;

    protected:
        INIT_BUILDER_STUB(Physics_Module_2D);

    };

}
