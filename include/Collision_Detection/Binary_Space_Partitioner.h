#pragma once

#include <L_Debug/L_Debug.h>
#include <Data_Structures/AVL_Tree.h>

#include <Collision_Detection/Broad_Phase_Interface.h>

namespace LPhys
{

    class Binary_Space_Partitioner : public Broad_Phase_Interface
    {
    private:
        unsigned int m_precision = 0;

    private:
        using Colliding_Pair_Tree = LDS::AVL_Tree<Colliding_Pair>;
        using Colliding_Point_And_Object_Tree = LDS::AVL_Tree<Colliding_Point_And_Object>;

    private:
        Colliding_Pair_Tree m_possible_collisions_tree;
        Colliding_Point_And_Object_Tree m_possible_collisions_with_points_tree;

    private:
        Border M_calculate_rb(const Objects_List& _objects_inside);
        Objects_List M_get_objects_inside_area(const Border& _rb, const Objects_List& _objects_maybe_inside);
        Points_List M_get_points_inside_area(const Border& _rb, const Points_List& _points_maybe_inside);
        void M_save_possible_collisions(const Objects_List& _objects_inside, const Points_List &_points_inside);
        void M_find_possible_collisions_in_area(const Border& _rb, const Objects_List& _objects_inside, const Points_List &_points_inside, unsigned int _same_objects_repetition);

    public:
        inline void set_precision(unsigned int _precision) { m_precision = _precision; }

    public:
        void update(const Objects_List &_registred_objects, const Points_List &_registred_points) override;

    };

}
