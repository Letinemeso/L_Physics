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

void Binary_Space_Partitioner::M_save_possible_collisions(const Temp_Objects_Container& _objects_inside)
{
    if(_objects_inside.size() < 2)
        return;

    for(unsigned int i_1 = 0; i_1 < _objects_inside.size(); ++i_1)
    {
        Physics_Module* pm_1 = _objects_inside[i_1];

        for(unsigned int i_2 = i_1 + 1; i_2 < _objects_inside.size(); ++i_2)
        {
            Physics_Module* pm_2 = _objects_inside[i_2];

            Colliding_Pair cp(pm_1, pm_2);

            if( m_possible_collisions_tree.find(cp).is_ok() )
                continue;

            if( ! (pm_1->may_intersect_with_other(*pm_2) && pm_2->may_intersect_with_other(*pm_1)) )
                continue;

            if(!passes_filters(pm_1, pm_2))
                continue;

            m_possible_collisions_tree.insert(cp);
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
    m_possible_collisions_tree.clear();
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

    for(Colliding_Pair_Tree::Iterator it = m_possible_collisions_tree.iterator(); !it.end_reached(); ++it)
        m_possible_collisions.push_back(*it);
}
