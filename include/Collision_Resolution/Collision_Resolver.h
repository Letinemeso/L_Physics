#pragma once

#include <L_Debug/L_Debug.h>
#include <Data_Structures/Map.h>

#include <Collision_Detection/Intersection_Data.h>
#include <Collision_Resolution/Collision_Resolution_Interface.h>


namespace LPhys
{

    class Collision_Resolver
    {
    private:
        using Collision_Resolutions_List = LDS::List<Collision_Resolution_Interface*>;
        using Collision_Resolutions_Map = LDS::Map<std::string, Collision_Resolution_Interface*>;

    private:
        Collision_Resolution_Interface* m_resolution = nullptr;

    public:
        Collision_Resolver();
        ~Collision_Resolver();

    private:
        Collision_Resolver(const Collision_Resolver& _other) = delete;
        Collision_Resolver(Collision_Resolver&& _other) = delete;

    public:
        inline void set_resolution(Collision_Resolution_Interface* _ptr) { delete m_resolution; m_resolution = _ptr; }

    public:
        void resolve_single(const Intersection_Data& _id, float _dt) const;
        void resolve_all(const LDS::List<Intersection_Data>& _ids, float _dt) const;

    };

}
