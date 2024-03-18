#pragma once

#include <L_Debug/L_Debug.h>
#include <Data_Structures/List.h>

#include <Collision_Detection/Broad_Phase/Broad_Phase_Interface.h>
#include <Collision_Detection/Narrow_Phase/Narrow_Phase_Interface.h>
#include <Modules/Physics_Module.h>


namespace LPhys
{
    class Collision_Detector final
	{
	public:
        Collision_Detector();
        ~Collision_Detector();

    private:
        Broad_Phase_Interface* m_broad_phase = nullptr;
        Narrow_Phase_Interface* m_narrow_phase = nullptr;

    public:
        using Registred_Modules_List = LDS::List<const Physics_Module*>;

    private:
        Registred_Modules_List m_registred_modules;

    public:
        using Intersection_Data_List = LDS::List<Intersection_Data>;

	public:
        void set_broad_phase(Broad_Phase_Interface* _broad_phase_impl);
        void set_narrow_phase(Narrow_Phase_Interface* _narrow_phase_impl);

	public:
        void register_module(const Physics_Module* _module);
        void unregister_module(const Physics_Module* _module);
        void unregister_all_modules();

    public:
        inline const Registred_Modules_List& registred_modules() const { return m_registred_modules; }

	public:
        void update();

	public:
        const Intersection_Data_List& found_collisions() const;

	};

}
