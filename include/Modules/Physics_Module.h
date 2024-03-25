#pragma once

#include <Module.h>

#include <Physical_Models/Border.h>


namespace LPhys
{

    class Physics_Module : public LEti::Module
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module, LEti::Module);

    public:
        using On_Collision_Function = LST::Function<void(const Physics_Module*)>;

    private:
        bool m_can_collide = true;

    private:
        On_Collision_Function m_on_collision_func;

    public:
        inline void set_on_collision_function(On_Collision_Function _func) { m_on_collision_func = _func; }
        inline void allow_collisions(bool _value) { m_can_collide = _value; }

    public:
        inline bool can_collide() const { return m_can_collide; }

    public:
        inline void on_collision(const Physics_Module* _with) const { if(m_on_collision_func) m_on_collision_func(_with); }

    public:
        virtual void expand_border(Border& _border) const;
        virtual bool may_intersect_with_other(const Physics_Module& _other) const;
        virtual bool intersects_with_border(const Border& _border) const;

    };


    class Physics_Module_Stub : public LEti::Module_Stub
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module_Stub, LEti::Module_Stub);

    };

}
