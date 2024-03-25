#include <Collision_Detection/Broad_Phase/Binary_Space_Partitioner.h>

using namespace LPhys;


Border Binary_Space_Partitioner::M_calculate_rb(const Objects_List& _objects_inside)
{
    Border result;

    for(Objects_List::Const_Iterator it = _objects_inside.begin(); !it.end_reached(); ++it)
    {
        if((*it)->can_collide())
            (*it)->expand_border(result);
    }

    return result;
}

Binary_Space_Partitioner::Objects_List Binary_Space_Partitioner::M_get_objects_inside_area(const Border& _rb, const Objects_List& _objects_maybe_inside)
{
    Objects_List result;

    for(Objects_List::Const_Iterator it = _objects_maybe_inside.begin(); !it.end_reached(); ++it)
    {
        if(!(*it)->can_collide())
            continue;

        if((*it)->intersects_with_border(_rb))
            result.push_back(*it);
    }

    return result;
}

void Binary_Space_Partitioner::M_save_possible_collisions(const Objects_List& _objects_inside)
{
    if(_objects_inside.size() < 2)
        return;

    for(Objects_List::Const_Iterator it_1 = _objects_inside.begin(); !it_1.end_reached(); ++it_1)
    {
        Objects_List::Const_Iterator it_2 = it_1;
        ++it_2;

        const Physics_Module* pm_1 = *it_1;

        for(; !it_2.end_reached(); ++it_2)
        {
            const Physics_Module* pm_2 = *it_2;

            if( ! (pm_1->may_intersect_with_other(*pm_2) && pm_2->may_intersect_with_other(*pm_1)) )
                continue;

            Colliding_Pair cp(*it_1, *it_2);

            if( m_possible_collisions_tree.find(cp).is_ok() )
                continue;

            m_possible_collisions_tree.insert(cp);
        }
    }
}

void Binary_Space_Partitioner::M_find_possible_collisions_in_area(const Border& _rb, const Objects_List& _objects_inside, unsigned int _same_objects_repetition)
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

    Objects_List objects_inside_1 = M_get_objects_inside_area(rb_1, _objects_inside);
    Objects_List objects_inside_2 = M_get_objects_inside_area(rb_2, _objects_inside);

    M_find_possible_collisions_in_area( rb_1, objects_inside_1, (objects_inside_1.size() == _objects_inside.size() ? _same_objects_repetition + 1 : 0) );
    M_find_possible_collisions_in_area( rb_2, objects_inside_2, (objects_inside_2.size() == _objects_inside.size() ? _same_objects_repetition + 1 : 0) );
}



void Binary_Space_Partitioner::update(const Objects_List &_registred_objects)
{
    m_possible_collisions.clear();

    m_possible_collisions_tree.clear();

    M_find_possible_collisions_in_area(M_calculate_rb(_registred_objects), _registred_objects, 0);

    for(Colliding_Pair_Tree::Iterator it = m_possible_collisions_tree.iterator(); !it.end_reached(); ++it)
        m_possible_collisions.push_back(*it);
}
