#include <Collision_Detection/Primitives/SAT_Models_Intersection_2D.h>

using namespace LPhys;


void SAT_Models_Intersection_2D::M_rotate_2D_vector_perpendicular(glm::vec3& _vec) const
{
    float x = _vec.x;
    _vec.x = -_vec.y;
    _vec.y = x;
}


SAT_Models_Intersection_2D::MinMax_Pair SAT_Models_Intersection_2D::M_get_minmax_projections(const glm::vec3 &_axis, const Polygon &_pol) const
{
    MinMax_Pair result;

    result.min = LEti::Math::dot_product(_pol[0], _axis);      //  point to axis projection
    result.max = result.min;

	for(unsigned int i=1; i<3; ++i)
	{
        float proj = LEti::Math::dot_product(_pol[i], _axis);
        if(proj < result.min)
            result.min = proj;
        if(proj > result.max)
            result.max = proj;
	}

    return result;
}

float SAT_Models_Intersection_2D::M_point_to_segment_distance(const glm::vec3& _point, const glm::vec3& _seg_start, const glm::vec3& _seg_end) const
{
    glm::vec3 segment_direction_vec = _seg_end - _seg_start;
    glm::vec3 point_to_start_vec = _point - _seg_start;

    glm::vec3 cross = LEti::Math::cross_product(point_to_start_vec, segment_direction_vec);

    float cross_length = LEti::Math::vector_length(cross);
    float segment_length = LEti::Math::vector_length(segment_direction_vec);

    float distance = cross_length / segment_length;

    return distance;
}


SAT_Models_Intersection_2D::Intersection_Data SAT_Models_Intersection_2D::M_polygons_collision(const Polygon &_first, const Polygon &_second) const
{
    SAT_Models_Intersection_2D::Intersection_Data result;

	for(unsigned int i=0; i<3; ++i)
    {
        if(!_first.segment_can_collide(i))
            continue;

		glm::vec3 axis = _first[i + 1] - _first[i];
        axis.z = 0.0f;  //  this should fix float calculation inaccuracy for 2D objects
        LEti::Math::shrink_vector_to_1(axis);
        M_rotate_2D_vector_perpendicular(axis);

        MinMax_Pair f = M_get_minmax_projections(axis, _first);
        MinMax_Pair s = M_get_minmax_projections(axis, _second);

        if(f.min > s.max || s.min > f.max)
            return Intersection_Data();

		result.intersection = true;

        float dist_1 = fabs(f.max - s.min);
        float dist_2 = fabs(s.max - f.min);

		float min = dist_1 < dist_2 ? dist_1 : dist_2;

		if(result.min_dist < 0.0f || result.min_dist > min)
		{
			result.min_dist = min;
			result.min_dist_axis = axis;
		}
	}

    for(unsigned int i=0; i<3; ++i)
    {
        if(!_second.segment_can_collide(i))
            continue;

        glm::vec3 axis = _second[i + 1] - _second[i];
        axis.z = 0.0f;  //  this should fix float calculation inaccuracy for 2D objects
        LEti::Math::shrink_vector_to_1(axis);
        M_rotate_2D_vector_perpendicular(axis);

        MinMax_Pair f = M_get_minmax_projections(axis, _second);
        MinMax_Pair s = M_get_minmax_projections(axis, _first);

        if(f.min > s.max || s.min > f.max)
            return Intersection_Data();

        result.intersection = true;

        float dist_1 = fabs(f.max - s.min);
        float dist_2 = fabs(s.max - f.min);

        float min = dist_1 < dist_2 ? dist_1 : dist_2;

        if(result.min_dist < 0.0f || result.min_dist > min)
        {
            result.min_dist = min;
            result.min_dist_axis = axis;
        }
    }

	glm::vec3 direction = _second.center() - _first.center();

	if (LEti::Math::dot_product(direction, result.min_dist_axis) < 0.0f)
		result.min_dist_axis = -result.min_dist_axis;

	return result;
}


float SAT_Models_Intersection_2D::M_smallest_point_to_polygon_distance(const glm::vec3 &_point, const Polygon &_pol) const
{
	float min_dist = -1.0f;

	for(unsigned int i=0; i<3; ++i)
	{
		if(_pol.segment_can_collide(i) == false)
			continue;

        float min = M_point_to_segment_distance(_point, _pol[i], _pol[i + 1]);

		if(min < 0.0f)
			continue;

		if(min_dist < 0.0f || min_dist > min)
			min_dist = min;
	}

	return min_dist;
}

LDS::List<glm::vec3> SAT_Models_Intersection_2D::M_points_of_contact(const Polygon_Holder_Base* _f_pols, unsigned int _f_count, const Polygon_Holder_Base* _s_pols, unsigned int _s_count) const
{
	float min_dist = -1.0f;

	LDS::List<glm::vec3> result;

	auto point_already_counted = [&](const glm::vec3& _point)->bool
	{
		for(LDS::List<glm::vec3>::Iterator it = result.begin(); !it.end_reached(); ++it)
            if(LEti::Math::vecs_are_equal(*it, _point))
				return true;
		return false;
	};

	for(unsigned int i=0; i<_f_count; ++i)
	{
        const Polygon& cur_pol = *_f_pols->get_polygon(i);

		for(unsigned int j=0; j<3; ++j)
		{
			const glm::vec3& cur_point = cur_pol[j];

			for(unsigned int s_pol_i = 0; s_pol_i < _s_count; ++s_pol_i)
			{
                float min = M_smallest_point_to_polygon_distance(cur_point, *_s_pols->get_polygon(s_pol_i));

				if(min < 0.0f)
					continue;

				if(min_dist < 0.0f || min_dist > min)
				{
					result.clear();
					min_dist = min;
					result.push_back(cur_point);
				}
                else if(LEti::Math::floats_are_equal(min_dist, min))
				{
					if(point_already_counted(cur_point))
						continue;
					result.push_back(cur_point);
				}
			}
		}
	}

	for(unsigned int i=0; i<_s_count; ++i)
	{
        const Polygon& cur_pol = *_s_pols->get_polygon(i);

		for(unsigned int j=0; j<3; ++j)
		{
			const glm::vec3& cur_point = cur_pol[j];

			for(unsigned int f_pol_i = 0; f_pol_i < _f_count; ++f_pol_i)
			{
                float min = M_smallest_point_to_polygon_distance(cur_point, *_f_pols->get_polygon(f_pol_i));

				if(min < 0.0f)
					continue;

				if(min_dist < 0.0f || min_dist > min)
				{
					result.clear();
					min_dist = min;
					result.push_back(cur_point);
				}
                else if(LEti::Math::floats_are_equal(min_dist, min))
				{
					if(point_already_counted(cur_point))
						continue;
					result.push_back(cur_point);
				}
			}
		}
	}

	return result;
}



LPhys::Intersection_Data SAT_Models_Intersection_2D::collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _pols_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _pols_amount_2) const
{
    unsigned int first_collided_polygon = 0xFFFFFFFF;
    unsigned int second_collided_polygon = 0xFFFFFFFF;

    glm::vec3 push_out_vector(0.0f, 0.0f, 0.0f);

    for(unsigned int p_0 = 0; p_0 < _pols_amount_1; ++p_0)
	{
        const Polygon& first_polygon = *_polygon_holder_1->get_polygon(p_0);

        for(unsigned int p_1 = 0; p_1 < _pols_amount_2; ++p_1)
        {
            const Polygon& second_polygon = *_polygon_holder_2->get_polygon(p_1);

            SAT_Models_Intersection_2D::Intersection_Data id = M_polygons_collision(first_polygon, second_polygon);

			if(!id.intersection)
				continue;

            glm::vec3 local_push_out_vector = id.min_dist_axis;
            LEti::Math::extend_vector_to_length(local_push_out_vector, id.min_dist);

            for(unsigned int i = 0; i < 3; ++i)
            {
                if(fabsf(push_out_vector[i]) >= fabsf(local_push_out_vector[i]))
                    continue;

                push_out_vector[i] = local_push_out_vector[i];
                first_collided_polygon = p_0;
                second_collided_polygon = p_1;
            }
		}
    }

    if(first_collided_polygon == 0xFFFFFFFF)
        return {};

    float depth = LEti::Math::vector_length(push_out_vector);
    if(depth > 0.00001f)
        push_out_vector /= depth;

    LPhys::Intersection_Data result(LPhys::Intersection_Data::Type::intersection);

    result.normal = -push_out_vector;
	LEti::Math::shrink_vector_to_1(result.normal);
    result.depth = depth;

    LDS::List<glm::vec3> points = M_points_of_contact(_polygon_holder_1, _pols_amount_1, _polygon_holder_2, _pols_amount_2);
	if(points.size() == 0)
        return {};

	for(LDS::List<glm::vec3>::Iterator it = points.begin(); !it.end_reached(); ++it)
		result.point += *it;

	result.point /= (float)points.size();

    result.first_collided_polygon_index = first_collided_polygon;
    result.second_collided_polygon_index = second_collided_polygon;

	return result;
}






























































