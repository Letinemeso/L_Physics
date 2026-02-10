#include <Collision_Detection/Broad_Phase/Binary_Space_Partitioner.h>

using namespace LPhys;


unsigned int Binary_Space_Partitioner::M_calculate_exclusions_amount(unsigned int _elements_amount) const
{
    if(_elements_amount < 2)
        return 0;

    return ( _elements_amount * ( _elements_amount - 1 ) ) / 2;
}

unsigned int Binary_Space_Partitioner::M_calculate_exclusion_index(unsigned int _first_index, unsigned int _second_index) const
{
    L_ASSERT(_first_index != _second_index);

    if(_first_index > _second_index)
        std::swap(_first_index, _second_index);

    return (_first_index * m_registred_objects.size()) - M_calculate_exclusions_amount(_first_index + 1) + (_second_index - _first_index - 1);
}

bool Binary_Space_Partitioner::M_already_checked(unsigned int _first_index, unsigned int _second_index) const
{
    unsigned int exclusion_index = M_calculate_exclusion_index(_first_index, _second_index);
    return m_exclusions[exclusion_index];
}

void Binary_Space_Partitioner::M_mark_for_exclusion(unsigned int _first_index, unsigned int _second_index)
{
    unsigned int exclusion_index = M_calculate_exclusion_index(_first_index, _second_index);
    L_ASSERT(!m_exclusions[exclusion_index]);
    m_exclusions[exclusion_index] = true;
}



Border Binary_Space_Partitioner::M_calculate_rb(const Temp_Objects_Container& _objects_inside)
{
    Border result;

    for(unsigned int i=0; i<_objects_inside.size(); ++i)
    {
        if(_objects_inside[i].module->can_collide() || m_ignore_modules_collision_restriction)
            _objects_inside[i].module->expand_border(result);
    }

    return result;
}

Binary_Space_Partitioner::Indices_Stack::Scope Binary_Space_Partitioner::M_add_objects_inside_to_stack(const Border& _rb, const Indices_Stack::Scope& _objects_maybe_inside)
{
    m_indices_stack.remember_size();

    for(unsigned int i = 0; i < _objects_maybe_inside.size(); ++i)
    {
        const Module_ID_Wrapper& module_wrapper = m_registred_objects[_objects_maybe_inside[i]];

        if(!module_wrapper.module->can_collide() && !m_ignore_modules_collision_restriction)
            continue;

        if(module_wrapper.module->intersects_with_border(_rb))
            m_indices_stack.push(_objects_maybe_inside[i]);
    }

    return m_indices_stack.scope();
}

glm::vec3 Binary_Space_Partitioner::M_calculate_border_modifier(const Border& _rb) const
{
    glm::vec3 modifier = _rb.size() * 0.5f;

    if(modifier.x > modifier.y)                 //  that's some ugly shit, but doing cycles would be overkill
    {
        if(modifier.x > modifier.z)
            modifier = {modifier.x, 0.0f, 0.0f};
        else
            modifier = {0.0f, 0.0f, modifier.z};
    }
    else
    {
        if(modifier.y > modifier.z)
            modifier = {0.0f, modifier.y, 0.0f};
        else
            modifier = {0.0f, 0.0f, modifier.z};
    }

    return modifier;
}

bool Binary_Space_Partitioner::M_object_lists_same(const Indices_Stack::Scope& _first, const Indices_Stack::Scope& _second) const
{
    if(_first.size() != _second.size())
        return false;

    for(unsigned int i = 0; i < _first.size(); ++i)
    {
        if(_first[i] != _second[i])
            return false;
    }

    return true;
}

void Binary_Space_Partitioner::M_save_possible_collisions(const Indices_Stack::Scope& _objects_inside)
{
    if(_objects_inside.size() < 2)
        return;

    Colliding_Pair cp;

    for(unsigned int i_1 = 0; i_1 < _objects_inside.size(); ++i_1)
    {
        const Module_ID_Wrapper& first_wrapper = m_registred_objects[_objects_inside[i_1]];
        cp.first = first_wrapper.module;

        for(unsigned int i_2 = i_1 + 1; i_2 < _objects_inside.size(); ++i_2)
        {
            const Module_ID_Wrapper& second_wrapper = m_registred_objects[_objects_inside[i_2]];
            cp.second = second_wrapper.module;

            if(M_already_checked(first_wrapper.id, second_wrapper.id))
                continue;

            M_mark_for_exclusion(first_wrapper.id, second_wrapper.id);

            if( ! (cp.first->may_intersect_with_other(*cp.second) && cp.second->may_intersect_with_other(*cp.first)) )
                continue;

            if(!passes_filters(cp.first, cp.second))
                continue;

            m_possible_collisions.push_back(cp);
        }
    }
}

void Binary_Space_Partitioner::M_find_possible_collisions_in_area(const Border& _rb, const Indices_Stack::Scope& _objects_inside, unsigned int _recursion_level, unsigned int _same_objects_repetition)
{
    if( (_recursion_level > m_max_recursion_level) || (_same_objects_repetition == m_precision) || (_objects_inside.size() <= 2) )
        return M_save_possible_collisions(_objects_inside);

    Border rb_1 = _rb;
    Border rb_2 = _rb;

    glm::vec3 modifier = M_calculate_border_modifier(_rb);

    rb_1.modify_size(-modifier);
    rb_2.modify_size(-modifier);
    rb_2.modify_offset(modifier);

    auto calculate_repetitions_counter = [this](const Indices_Stack::Scope& _old_objects, const Indices_Stack::Scope& _new_objects, unsigned int _current_counter)->unsigned int
    {
        if(M_object_lists_same(_old_objects, _new_objects))
            return _current_counter + 1;
        return 0;
    };

    Indices_Stack::Scope objects_inside_1 = M_add_objects_inside_to_stack(rb_1, _objects_inside);
    unsigned int repetitions_counter_1 = calculate_repetitions_counter(_objects_inside, objects_inside_1, _same_objects_repetition);
    M_find_possible_collisions_in_area( rb_1, objects_inside_1, _recursion_level + 1, repetitions_counter_1 );
    m_indices_stack.clear_scope(objects_inside_1);

    Indices_Stack::Scope objects_inside_2 = M_add_objects_inside_to_stack(rb_2, _objects_inside);
    unsigned int repetitions_counter_2 = calculate_repetitions_counter(_objects_inside, objects_inside_2, _same_objects_repetition);
    M_find_possible_collisions_in_area( rb_2, objects_inside_2, _recursion_level + 1, repetitions_counter_2 );
    m_indices_stack.clear_scope(objects_inside_2);
}



void Binary_Space_Partitioner::reset()
{
    m_registred_objects.clear();
    m_possible_collisions.clear();
}

void Binary_Space_Partitioner::add_models(const Objects_List& _objects)
{
    m_registred_objects.resize(m_registred_objects.size() + _objects.size());
    for(Objects_List::Const_Iterator it = _objects.begin(); !it.end_reached(); ++it)
        m_registred_objects.push({ *it, m_registred_objects.size() });
}

void Binary_Space_Partitioner::process()
{
    if(m_registred_objects.size() < 2)
        return;

    m_exclusions_size = M_calculate_exclusions_amount(m_registred_objects.size());
    m_exclusions = new bool[m_exclusions_size]{false};

    m_indices_stack.remember_size();
    for(unsigned int i = 0; i < m_registred_objects.size(); ++i)
        m_indices_stack.push(i);

    Border initial_border = M_calculate_rb(m_registred_objects);
    Indices_Stack::Scope objects_in_initial_border = m_indices_stack.scope();

    M_find_possible_collisions_in_area(initial_border, objects_in_initial_border, 0, 0);

    m_indices_stack.clear_scope(objects_in_initial_border);

    delete[] m_exclusions;
}
