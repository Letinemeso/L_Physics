#include <Collision_Detection/Broad_Phase/Binary_Space_Partitioner.h>

using namespace LPhys;


Border Binary_Space_Partitioner::M_calculate_rb(const Temp_Objects_Container& _objects_inside)
{
    Border result;

    for(unsigned int i=0; i<_objects_inside.size(); ++i)
    {
        if(_objects_inside[i]->can_collide() || m_ignore_modules_collision_restriction)
            _objects_inside[i]->expand_border(result);
    }

    return result;
}

Binary_Space_Partitioner::Temp_Objects_Container Binary_Space_Partitioner::M_get_objects_inside_area(const Border& _rb, const Temp_Objects_Container& _objects_maybe_inside)
{
    Temp_Objects_Container result(_objects_maybe_inside.size());

    for(unsigned int i=0; i<_objects_maybe_inside.size(); ++i)
    {
        if(!_objects_maybe_inside[i]->can_collide() && !m_ignore_modules_collision_restriction)
            continue;

        if(_objects_maybe_inside[i]->intersects_with_border(_rb))
            result.push(_objects_maybe_inside[i]);
    }

    return result;
}

Binary_Space_Partitioner::Exclusion_Tree& Binary_Space_Partitioner::M_get_exclusion_tree(const Physics_Module* _for)
{
    Exclusion_Trees_Map::Iterator exclusion_tree_it = m_exclusion_trees.find(_for);
    if(!exclusion_tree_it.is_ok())
        exclusion_tree_it = m_exclusion_trees.insert_and_get_iterator(_for, {});

    return *exclusion_tree_it;
}

void Binary_Space_Partitioner::M_save_possible_collisions(const Temp_Objects_Container& _objects_inside)
{
    if(_objects_inside.size() < 2)
        return;

    Colliding_Pair cp;

    for(unsigned int i_1 = 0; i_1 < _objects_inside.size(); ++i_1)
    {
        cp.first = _objects_inside[i_1];

        Exclusion_Tree& exclusion_tree_first = M_get_exclusion_tree(cp.first);

        for(unsigned int i_2 = i_1 + 1; i_2 < _objects_inside.size(); ++i_2)
        {
            cp.second = _objects_inside[i_2];

            Exclusion_Tree::Iterator maybe_excluded_it = exclusion_tree_first.find(cp.second);
            if(maybe_excluded_it.is_ok())
                continue;
            exclusion_tree_first.insert(cp.second);

            Exclusion_Tree& exclusion_tree_second = M_get_exclusion_tree(cp.second);
            maybe_excluded_it = exclusion_tree_second.find(cp.first);
            if(maybe_excluded_it.is_ok())
                continue;
            exclusion_tree_second.insert(cp.first);


            if( ! (cp.first->may_intersect_with_other(*cp.second) && cp.second->may_intersect_with_other(*cp.first)) )
                continue;

            if(!passes_filters(cp.first, cp.second))
                continue;

            m_possible_collisions.push_back(cp);
        }
    }
}

void Binary_Space_Partitioner::M_find_possible_collisions_in_area(const Border& _rb, const Temp_Objects_Container& _objects_inside, unsigned int _same_objects_repetition)
{
    if( (_same_objects_repetition == m_precision) || (_objects_inside.size() <= 2) )
        return M_save_possible_collisions(_objects_inside);

    Border rb_1 = _rb;
    Border rb_2 = _rb;

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

    rb_1.modify_size(-modifier);
    rb_2.modify_size(-modifier);
    rb_2.modify_offset(modifier);

    Temp_Objects_Container objects_inside_1 = M_get_objects_inside_area(rb_1, _objects_inside);
    Temp_Objects_Container objects_inside_2 = M_get_objects_inside_area(rb_2, _objects_inside);

    M_find_possible_collisions_in_area( rb_1, objects_inside_1, (objects_inside_1.size() == _objects_inside.size() ? _same_objects_repetition + 1 : 0) );
    M_find_possible_collisions_in_area( rb_2, objects_inside_2, (objects_inside_2.size() == _objects_inside.size() ? _same_objects_repetition + 1 : 0) );
}



void Binary_Space_Partitioner::reset()
{
    m_registred_objects.clear();
    m_exclusion_trees.clear();
    m_possible_collisions.clear();
}

void Binary_Space_Partitioner::add_models(const Objects_List& _objects)
{
    m_registred_objects.resize(m_registred_objects.size() + _objects.size());
    for(Objects_List::Const_Iterator it = _objects.begin(); !it.end_reached(); ++it)
        m_registred_objects.push(*it);
}

void Binary_Space_Partitioner::process()
{
    M_find_possible_collisions_in_area(M_calculate_rb(m_registred_objects), m_registred_objects, 0);
}
