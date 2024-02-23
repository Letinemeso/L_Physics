#pragma once

#include <L_Debug/L_Debug.h>
#include <Data_Structures/List.h>

#include <Collision_Detection/Broad_Phase/Broad_Phase_Interface.h>
#include <Collision_Detection/Narrow_Phase/Narrow_Phase_Interface.h>
#include <Modules/Physics_Module.h>


namespace LPhys
{
    class Collision_Detector_2D final
	{
	public:
		Collision_Detector_2D();
        ~Collision_Detector_2D();

	private:
        using Registred_Modules_List = LDS::List<const Physics_Module*>;
        Registred_Modules_List m_registred_modules;

	private:
        void debug_assert_if_model_copy_found(const Physics_Module *_model, bool _reverse);

	private:
		Broad_Phase_Interface* m_broad_phase = nullptr;
		Narrow_Phase_Interface* m_narrow_phase = nullptr;
        Narrowest_Phase_Interface* m_narrowest_phase = nullptr;

	public:
        void set_broad_phase(Broad_Phase_Interface* _broad_phase_impl);
        void set_narrow_phase(Narrow_Phase_Interface* _narrow_phase_impl);
		void set_narrowest_phase(Narrowest_Phase_Interface* _narrowest_phase_impl);

	public:
        void register_object(const Physics_Module* _model);
        void unregister_object(const Physics_Module* _model);
        void unregister_all_objects();

	public:
        void update();

	public:
        const Narrow_Phase_Interface::Collision_Data_List__Models& get_collisions__models();

	};

}
