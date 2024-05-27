#pragma once

#include <L_Debug/L_Debug.h>
#include <Data_Structures/AVL_Tree.h>

#include <Collision_Detection/Broad_Phase/Broad_Phase_Interface.h>

namespace LPhys
{

    class Binary_Space_Partitioner : public Broad_Phase_Interface
    {
    private:
        unsigned int m_precision = 3;

    private:
        using Colliding_Pair_Tree = LDS::AVL_Tree<Colliding_Pair>;

    private:
        Objects_List m_registred_objects;
        Colliding_Pair_Tree m_possible_collisions_tree;

    private:
        Border M_calculate_rb(const Objects_List& _objects_inside);
        Objects_List M_get_objects_inside_area(const Border& _rb, const Objects_List& _objects_maybe_inside);
        void M_save_possible_collisions(const Objects_List& _objects_inside);
        void M_find_possible_collisions_in_area(const Border& _rb, const Objects_List& _objects_inside, unsigned int _same_objects_repetition);

    public:
        inline void set_precision(unsigned int _precision) { m_precision = _precision; }

    public:
        void reset() override;
        void add_models(const Objects_List& _objects) override;
        void process() override;

    };

}
