#pragma once

#include <L_Debug/L_Debug.h>
#include <Data_Structures/Vector.h>

#include <Collision_Detection/Intersection_Data.h>
#include <Collision_Resolution/Collision_Resolution_Interface.h>


namespace LPhys
{

    class Collision_Resolver
    {
    private:
        using Collision_Resolutions = LDS::Vector<Collision_Resolution_Interface*>;

    private:
        Collision_Resolutions m_resolutions;

    public:
        Collision_Resolver();
        ~Collision_Resolver();

    private:
        Collision_Resolver(const Collision_Resolver& _other) = delete;
        Collision_Resolver(Collision_Resolver&& _other) = delete;

    public:
        inline void add_resolution(Collision_Resolution_Interface* _ptr) { m_resolutions.push(_ptr); }

        void clear_resolutions();

    public:
        void resolve_single(const Intersection_Data& _id, float _dt) const;
        void resolve_all(const LDS::List<Intersection_Data>& _ids, float _dt) const;

    };

}
