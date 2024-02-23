#pragma once

#include <Module.h>

#include <Physical_Models/Border.h>


namespace LPhys
{

    class Physics_Module : public LEti::Module
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module, LEti::Module);

    protected:
        Border m_border;

    public:
        inline const Border& border() const { return m_border; }

    public:
        virtual bool may_intersect_with_other(const Physics_Module& _other) const;
        virtual bool intersects_with_border(const Border& _border) const;

    };


    class Physics_Module_Stub : public LEti::Module_Stub
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module, LEti::Module);

    };

}
