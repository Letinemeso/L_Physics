#include <Collision_Detection/Binary_Space_Partitioner.h>

using namespace LPhys;


Border Binary_Space_Partitioner::M_calculate_rb(const Objects_List& _objects_inside)
{
    Border result;

    for(Objects_List::Const_Iterator it = _objects_inside.begin(); !it.end_reached(); ++it)
        result = result || (*it)->border();

    return result;
}

Binary_Space_Partitioner::Objects_List Binary_Space_Partitioner::M_get_objects_inside_area(const Border& _rb, const Objects_List& _objects_maybe_inside)
{
    Objects_List result;

    for(Objects_List::Const_Iterator it = _objects_maybe_inside.begin(); !it.end_reached(); ++it)
    {
        if(_rb && (*it)->border())
            result.push_back(*it);
    }

    return result;
}

Binary_Space_Partitioner::Points_List Binary_Space_Partitioner::M_get_points_inside_area(const Border& _rb, const Points_List& _points_maybe_inside)
{
    Points_List result;

    for(Points_List::Const_Iterator it = _points_maybe_inside.begin(); !it.end_reached(); ++it)
    {
        if(_rb.point_is_inside(**it))
            result.push_back(*it);
    }

    return result;
}

void Binary_Space_Partitioner::M_save_possible_collisions(const Objects_List& _objects_inside, const Points_List &_points_inside)
{
    if(_objects_inside.size() < 2)
        return;

    for(Objects_List::Const_Iterator it_1 = _objects_inside.begin(); !it_1.end_reached(); ++it_1)
    {
        Objects_List::Const_Iterator it_2 = it_1;
        ++it_2;

        for(; !it_2.end_reached(); ++it_2)
        {
            if( ! ((*it_1)->border() && (*it_2)->border()) )
                continue;

            Colliding_Pair cp(*it_1, *it_2);

            if( m_possible_collisions_tree.find(cp).is_ok() )
                continue;

            m_possible_collisions_tree.insert(cp);
        }

        for(Points_List::Const_Iterator points_it = _points_inside.begin(); !points_it.end_reached(); ++points_it)
        {
            if( ! ((*it_1)->border().point_is_inside(**points_it)) )
                continue;

            Colliding_Point_And_Object cp(*it_1, *points_it);

            if( m_possible_collisions_with_points_tree.find(cp).is_ok() )
                continue;

            m_possible_collisions_with_points_tree.insert(cp);
        }
    }
}

void Binary_Space_Partitioner::M_find_possible_collisions_in_area(const Border& _rb, const Objects_List& _objects_inside, const Points_List &_points_inside, unsigned int _same_objects_repetition)
{
    if( (_same_objects_repetition == m_precision) || (_objects_inside.size() <= 2) )
    {
        M_save_possible_collisions(_objects_inside, _points_inside);
        return;
    }

    Border rb_1 = _rb;
    Border rb_2 = _rb;

    float current_rb_width_half = _rb.size().x * 0.5f;
    float current_rb_height_half = _rb.size().y * 0.5f;

    if(current_rb_width_half > current_rb_height_half)
    {
        rb_1.modify_size({-current_rb_width_half, 0.0f, 0.0f});
        rb_2.modify_size({-current_rb_width_half, 0.0f, 0.0f});

        rb_2.modify_offset({current_rb_width_half, 0.0f, 0.0f});
    }
    else
    {
        rb_1.modify_size({0.0f, -current_rb_height_half, 0.0f});
        rb_2.modify_size({0.0f, -current_rb_height_half, 0.0f});

        rb_2.modify_offset({0.0f, current_rb_height_half, 0.0f});
    }

    Objects_List objects_inside_1 = M_get_objects_inside_area(rb_1, _objects_inside);
    Objects_List objects_inside_2 = M_get_objects_inside_area(rb_2, _objects_inside);

    Points_List points_inside_1 = M_get_points_inside_area(rb_1, _points_inside);
    Points_List points_inside_2 = M_get_points_inside_area(rb_2, _points_inside);

    M_find_possible_collisions_in_area( rb_1, objects_inside_1, points_inside_1, (objects_inside_1.size() == _objects_inside.size() ? _same_objects_repetition + 1 : 0) );
    M_find_possible_collisions_in_area( rb_2, objects_inside_2, points_inside_2, (objects_inside_2.size() == _objects_inside.size() ? _same_objects_repetition + 1 : 0) );
}



void Binary_Space_Partitioner::update(const Objects_List &_registred_objects, const Points_List &_registred_points)
{
    m_possible_collisions__models.clear();
    m_possible_collisions__points.clear();

    m_possible_collisions_tree.clear();
    m_possible_collisions_with_points_tree.clear();

    M_find_possible_collisions_in_area(M_calculate_rb(_registred_objects), _registred_objects, _registred_points, 0);

    for(Colliding_Pair_Tree::Iterator it = m_possible_collisions_tree.iterator(); !it.end_reached(); ++it)
        m_possible_collisions__models.push_back(*it);
    for(Colliding_Point_And_Object_Tree::Iterator it = m_possible_collisions_with_points_tree.iterator(); !it.end_reached(); ++it)
        m_possible_collisions__points.push_back(*it);
}
