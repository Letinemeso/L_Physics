#pragma once

#include <Data_Structures/List.h>

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
        using Transformations_List = LDS::List<LEti::Transformation_Data>;

        Transformations_List m_transformations_after_collisions;

    private:
        On_Collision_Function m_on_collision_func;

    protected:
        virtual void M_can_collide_changed() { }

    public:
        inline void set_on_collision_function(On_Collision_Function _func) { m_on_collision_func = _func; }
        inline void allow_collisions(bool _value) { m_can_collide = _value; M_can_collide_changed(); }
        inline void add_transformation_after_collision(const LEti::Transformation_Data& _data) { m_transformations_after_collisions.push_back(_data); }

    public:
        inline bool can_collide() const { return m_can_collide; }

    public:
        inline void on_collision(const Physics_Module* _with) const { if(m_on_collision_func) m_on_collision_func(_with); }

    public:
        virtual void expand_border(Border& _border) const;
        virtual bool may_intersect_with_other(const Physics_Module& _other) const;
        virtual bool intersects_with_border(const Border& _border) const;

    public:
        virtual void apply_data_after_collisions();

    };


    class Physics_Module_Stub : public LEti::Module_Stub
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module_Stub, LEti::Module_Stub);

        INIT_FIELDS
        ADD_FIELD(bool, allow_collisions)
        FIELDS_END

    public:
        bool allow_collisions = true;

    public:
        INIT_BUILDER_STUB(Physics_Module)

    };

}
