#pragma once

#include <Data_Structures/List.h>

#include <Module.h>

#include <Physical_Models/Border.h>
#include <Collision_Detection/Intersection_Data.h>


namespace LPhys
{

    class Physics_Module : public LEti::Module
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module, LEti::Module);

    public:
        using On_Collision_Function = LST::Function<void(const Physics_Module*, const Intersection_Data&)>;

    private:
        bool m_can_collide = true;
        bool m_is_static = false;

    protected:
        using Transformations_List = LDS::List<LEti::Transformation_Data>;
        Transformations_List m_transformations_after_collisions;

    private:
        On_Collision_Function m_on_collision_func;

    protected:
        virtual void M_can_collide_changed() { }

    public:
        inline void set_on_collision_function(On_Collision_Function _func) { m_on_collision_func = _func; }
        inline void allow_collisions(bool _value) { m_can_collide = _value; M_can_collide_changed(); }
        inline void set_static(bool _value) { m_is_static = _value; }
        inline void add_transformation_after_collision(const LEti::Transformation_Data& _data) { m_transformations_after_collisions.push_back(_data); }

    public:
        inline bool can_collide() const { return m_can_collide; }
        inline bool is_static() const { return m_is_static; }

    public:
        inline void on_collision(const Physics_Module* _with, const Intersection_Data& _id) const { if(m_on_collision_func) m_on_collision_func(_with, _id); }

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
        ADD_FIELD(bool, is_static)
        FIELDS_END

    public:
        bool allow_collisions = true;
        bool is_static = false;

    public:
        INIT_BUILDER_STUB(Physics_Module)

    };

}
