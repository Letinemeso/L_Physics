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
        Collision_Resolutions_List m_registred_resolutions;
        Collision_Resolutions_Map m_resolutions_map;

    public:
        Collision_Resolver();
        Collision_Resolver(const Collision_Resolver& _other) = delete;
        Collision_Resolver(Collision_Resolver&& _other) = delete;
        ~Collision_Resolver();

    public:

        void add_resolution(const std::string& _type_history_1, const std::string& _type_history_2, Collision_Resolution_Interface* _resolution);
        void clear_resolutions();

    public:
        void resolve_single(const Intersection_Data& _id, float _dt) const;
        void resolve_all(const LDS::List<Intersection_Data>& _ids, float _dt) const;

    };

}
