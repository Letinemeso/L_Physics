#ifndef __Dynamic_Physics_Module_2D
#define __Dynamic_Physics_Module_2D

#include <Modules/Physics_Module_Base.h>
#include <Physical_Models/Physical_Model_2D.h>


namespace LPhys
{

    class Dynamic_Physics_Module_2D_Stub : public Physics_Module_Base_Stub
    {
    public:
        DECLARE_VARIABLE;

    public:
        unsigned int coords_count = 0;
        float* coords = nullptr;

        bool* collision_permissions = nullptr;

    protected:
        LV::Variable_Base* M_construct_product() const override;
        void M_init_constructed_product(LV::Variable_Base* _product) const override;

    public:
        virtual ~Dynamic_Physics_Module_2D_Stub();
    };

    class Dynamic_Physics_Module_2D : public Physics_Module_Base
	{
    public:
        DECLARE_VARIABLE;

    private:
        Physical_Model_2D* m_physical_model = nullptr;
        Physical_Model_2D_Imprint* m_physical_model_prev_state = nullptr;
        LEti::Geometry_2D::Rectangular_Border m_rectangular_border;

	public:
        Dynamic_Physics_Module_2D();
        ~Dynamic_Physics_Module_2D();

    private:
        virtual Physical_Model_2D* M_create_physical_model() const;

    public:
	void init_physical_model(); //	allocates physical_model
	void init_prev_state();	    //	allocates physical_model_imprint with physical_model's data
	void setup_base_data(const float* _raw_coords, unsigned int _raw_coords_count, const bool* _collision_permissions);

        void move_raw(const glm::vec3 &_stride);

    public:
	void update_previous_state() override;
        void update(const glm::mat4x4 &_matrix) override;

	public:
        inline Physical_Model_2D* get_physical_model() { return m_physical_model; }
        inline Physical_Model_2D_Imprint* get_physical_model_prev_state() { return m_physical_model_prev_state; }
        inline const Physical_Model_2D* get_physical_model() const { return m_physical_model; }
        inline const Physical_Model_2D_Imprint* get_physical_model_prev_state() const { return m_physical_model_prev_state; }
        inline const LEti::Geometry_2D::Rectangular_Border& rectangular_border() const { return m_rectangular_border; }

	};

}


#endif // __Dynamic_Physics_Module_2D
