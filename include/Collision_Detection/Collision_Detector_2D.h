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
        Broad_Phase_Interface* m_broad_phase = nullptr;
        Narrow_Phase_Interface* m_narrow_phase = nullptr;

	private:
        using Registred_Modules_List = LDS::List<const Physics_Module*>;
        Registred_Modules_List m_registred_modules;

    public:
        using Intersection_Data_List = LDS::List<Intersection_Data>;

	public:
        void set_broad_phase(Broad_Phase_Interface* _broad_phase_impl);
        void set_narrow_phase(Narrow_Phase_Interface* _narrow_phase_impl);

	public:
        void register_object(const Physics_Module* _module);
        void unregister_object(const Physics_Module* _module);
        void unregister_all_objects();

	public:
        void update();

	public:
        const Intersection_Data_List& found_collisions() const;

	};

}
