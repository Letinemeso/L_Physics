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
        bool m_ignore_modules_collision_restriction = false;

    private:
        struct Module_ID_Wrapper
        {
            Physics_Module* module = nullptr;
            unsigned int id = 0;
        };

        using Temp_Objects_Container = LDS::Vector<Module_ID_Wrapper>;

    private:
        Temp_Objects_Container m_registred_objects;

        bool* m_exclusions = nullptr;           //  this can take quite a large amount of memory (~190 mb for 20000 registred physics modules), which is unlikely, but who knows
        unsigned int m_exclusions_size = 0;     //  possible optimization idea: store 'ints' instead and use separate bits as values to shrink down allocated memory 8 times

    private:
        unsigned int M_calculate_exclusions_amount(unsigned int _elements_amount) const;
        unsigned int M_calculate_exclusion_index(unsigned int _first_index, unsigned int _second_index) const;
        bool M_already_checked(unsigned int _first_index, unsigned int _second_index) const;
        void M_mark_for_exclusion(unsigned int _first_index, unsigned int _second_index);

        Border M_calculate_rb(const Temp_Objects_Container& _objects_inside);
        Temp_Objects_Container M_get_objects_inside_area(const Border& _rb, const Temp_Objects_Container& _objects_maybe_inside);
        glm::vec3 M_calculate_border_modifier(const Border& _rb) const;
        bool M_object_lists_same(const Temp_Objects_Container& _first, const Temp_Objects_Container& _second) const;
        void M_save_possible_collisions(const Temp_Objects_Container& _objects_inside);
        void M_find_possible_collisions_in_area(const Border& _rb, const Temp_Objects_Container& _objects_inside, unsigned int _same_objects_repetition);

    public:
        inline void set_precision(unsigned int _precision) { m_precision = _precision; }
        inline void ignore_modules_collision_restriction(bool _value) { m_ignore_modules_collision_restriction = _value; }

    public:
        void reset() override;
        void add_models(const Objects_List& _objects) override;
        void process() override;

    };

}
