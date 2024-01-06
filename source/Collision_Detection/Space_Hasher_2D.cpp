#include <Collision_Detection/Space_Hasher_2D.h>

using namespace LPhys;


unsigned int Space_Hasher_2D::get_number_binary_length(unsigned int _number)
{
	unsigned int result = 0;
	for(unsigned int i=0; i<sizeof(_number) * 8; ++i)
		if(_number & (1u << i))
			result = i + 1;
	return result;
}



unsigned int Space_Hasher_2D::construct_hash(unsigned int _x, unsigned int _y)
{
	return (_x << m_number_binary_length) | (_y);
}

void Space_Hasher_2D::update_border(const Objects_List& _registred_objects)
{
    if(_registred_objects.size() == 0)
        return;

    LDS::List<const Physics_Module_2D*>::Const_Iterator model_it = _registred_objects.begin();

    const LEti::Geometry_2D::Rectangular_Border* rb = &(*model_it)->rectangular_border();

    float max_left = rb->left;
    float max_right = rb->right;
    float max_top = rb->top;
    float max_bottom = rb->bottom;

    while(!model_it.end_reached())
    {
        rb = &(*model_it)->rectangular_border();

        if(rb->left < max_left)
            max_left = rb->left;
        if(rb->right > max_right)
            max_right = rb->right;
        if(rb->top > max_top)
            max_top = rb->top;
        if(rb->bottom < max_bottom)
            max_bottom = rb->bottom;

		++model_it;
	}

	m_space_borders.min_x = max_left;
	m_space_borders.min_y = max_bottom;
	m_space_borders.width = max_right - max_left;
	m_space_borders.height= max_top - max_bottom;
}

void Space_Hasher_2D::reset_hash_array()
{
	L_ASSERT(!(m_array == nullptr));

	for(unsigned int i=0; i<m_array_size; ++i)
	{
		if(m_array[i] != nullptr)
		{
			delete m_array[i];
			m_array[i] = nullptr;
		}
	}
}

void Space_Hasher_2D::hash_objects(const Objects_List& _registred_objects)
{
    Objects_List::Const_Iterator model_it = _registred_objects.begin();
    while(!model_it.end_reached())
    {
        const LEti::Geometry_2D::Rectangular_Border& curr_rb = (*model_it)->rectangular_border();

		unsigned int min_index_x = (unsigned int)((curr_rb.left - m_space_borders.min_x) / m_space_borders.width * m_precision);
		unsigned int max_index_x = (unsigned int)((curr_rb.right - m_space_borders.min_x) / m_space_borders.width * m_precision);
		unsigned int min_index_y = (unsigned int)((curr_rb.bottom - m_space_borders.min_y) / m_space_borders.height * m_precision);
		unsigned int max_index_y = (unsigned int)((curr_rb.top - m_space_borders.min_y) / m_space_borders.height * m_precision);

		for(unsigned int x = min_index_x; x <= max_index_x; ++x)
		{
			for(unsigned int y = min_index_y; y <= max_index_y; ++y)
			{
				unsigned int hash = construct_hash(x, y);

                if(!m_array[hash])
                {
                    m_array[hash] = new Objects_List;
                    m_array[hash]->push_back(*model_it);
                    continue;
                }

                bool copy = false;

                const Objects_List& list = *m_array[hash];
                for(Objects_List::Const_Iterator list_it = list.begin(); !list_it.end_reached(); ++list_it)
                {
                    if(*list_it == *model_it)
                        copy = true;
                }

                if(!copy)
                    m_array[hash]->push_back(*model_it);
			}
		}

		++model_it;
	}
}

void Space_Hasher_2D::check_for_possible_collisions__points(const Points_List &_registred_points)
{
    LDS::AVL_Tree<Colliding_Point_And_Object> possible_collisions;

    Points_List::Const_Iterator point_it = _registred_points.begin();
    while(!point_it.end_reached())
	{
		glm::vec3 point_with_offset = *(*point_it);
		point_with_offset.x -= m_space_borders.min_x;
		point_with_offset.y -= m_space_borders.min_y;

		if(point_with_offset.x < 0.0f || point_with_offset.y < 0.0f || point_with_offset.x > m_space_borders.width || point_with_offset.y > m_space_borders.height)
		{
			++point_it;
			continue;
		}

		unsigned int x = (unsigned int)(point_with_offset.x / m_space_borders.width * m_precision);
		unsigned int y = (unsigned int)(point_with_offset.y / m_space_borders.height * m_precision);
		unsigned int hash = construct_hash(x, y);

		if(m_array[hash] == nullptr)
		{
			++point_it;
			continue;
		}

        const Objects_List& list = *m_array[hash];
        Objects_List::Const_Iterator it = list.begin();
        while(!it.end_reached())
        {
            possible_collisions.insert(Colliding_Point_And_Object(*it, *point_it));
			++it;
		}

		++point_it;
	}

    m_possible_collisions__points.clear();

    LDS::AVL_Tree<Colliding_Point_And_Object>::Iterator it = possible_collisions.iterator();

    while(!it.end_reached())
    {
        m_possible_collisions__points.push_back({it->object, it->point});
        ++it;
    }
}



void Space_Hasher_2D::check_for_possible_collisions__models()
{
    LDS::AVL_Tree<Colliding_Pair> possible_collisions;  //  TODO: think about more optimal way to find already registred possible collisions

	for(unsigned int i=0; i<m_array_size; ++i)
	{
        if(m_array[i] == nullptr)
            continue;
		const Objects_List& curr_list = *(m_array[i]);
        if(curr_list.size() < 2)
            continue;

        Objects_List::Const_Iterator first = curr_list.begin();
        while(!first.end_reached())
		{
            Objects_List::Const_Iterator second = first;
			++second;

            while(!second.end_reached())
			{
				Colliding_Pair cd(*first, *second);
                if(!possible_collisions.find(cd).is_ok())
                    possible_collisions.insert(cd);

				++second;
			}

			++first;
		}
	}

    m_possible_collisions__models.clear();
    LDS::AVL_Tree<Colliding_Pair>::Iterator it = possible_collisions.iterator();

    while(!it.end_reached())
    {
        m_possible_collisions__models.push_back({it->first, it->second});
        ++it;
    }
}



Space_Hasher_2D::~Space_Hasher_2D()
{
	if(m_array)
	{
		for(unsigned int i=0; i<m_array_size; ++i)
			delete m_array[i];
		delete[] m_array;
	}
}



void Space_Hasher_2D::set_precision(unsigned int _precision)
{
	L_ASSERT(!(_precision == 0));

	if(m_array)
	{
		for(unsigned int i=0; i<m_array_size; ++i)
			delete m_array[i];
		delete[] m_array;
		m_array = nullptr;
	}

	m_number_binary_length = get_number_binary_length(_precision);
	m_precision = _precision;
	m_array_size = ((m_precision + 1) << m_number_binary_length) | m_precision + 1;
	m_array = new Objects_List*[m_array_size];
	for(unsigned int i=0; i<m_array_size; ++i)
		m_array[i] = nullptr;
}


void Space_Hasher_2D::update(const Objects_List &_registred_objects, const Points_List &_registred_points)
{
	update_border(_registred_objects);
	reset_hash_array();
	hash_objects(_registred_objects);
	check_for_possible_collisions__models();
	check_for_possible_collisions__points(_registred_points);
}
