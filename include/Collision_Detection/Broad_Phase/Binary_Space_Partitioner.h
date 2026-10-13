#pragma once

#include <L_Debug/L_Debug.h>
#include <Data_Structures/AVL_Tree.h>
#include <Stuff/Bounding_Volume_Hierarchy.h>

#include <Collision_Detection/Broad_Phase/Broad_Phase_Interface.h>

namespace LPhys
{

    class Binary_Space_Partitioner : public Broad_Phase_Interface   // TODO: replace internal logic with LST::Bounding_Volume_Hierarchy
    {
    private:
        LST::Bounding_Volume_Hierarchy m_bvh;

        bool m_ignore_modules_collision_restriction = false;

    private:
        LDS::Vector<Physics_Module*> m_registered_objects;
        LDS::Vector<LST::Border> m_registered_borders;

    public:
        inline void set_max_recursion_level(unsigned int _level) { m_bvh.set_max_depth(_level); }
        inline void ignore_modules_collision_restriction(bool _value) { m_ignore_modules_collision_restriction = _value; }

    public:
        void reset() override;
        void add_models(const Objects_List& _objects) override;
        void process() override;

    };

}
