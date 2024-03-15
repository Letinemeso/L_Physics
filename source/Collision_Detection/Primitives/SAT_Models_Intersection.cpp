#include <Collision_Detection/Primitives/SAT_Models_Intersection.h>

using namespace LPhys;


void SAT_Models_Intersection::M_rotate_2D_vector_perpendicular(glm::vec3& _vec) const
{
    float x = _vec.x;
    _vec.x = -_vec.y;
    _vec.y = x;
}


SAT_Models_Intersection::MinMax_Pair SAT_Models_Intersection::M_get_minmax_projections(const glm::vec3 &_axis, const Polygon &_pol) const
{
    MinMax_Pair result;

    result.min = LEti::Math::dot_product(_pol[0], _axis);      //  point to axis projection
    result.max = LEti::Math::dot_product(_pol[0], _axis);

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

float SAT_Models_Intersection::M_point_to_segment_distance(const glm::vec3& _point, const glm::vec3& _seg_start, const glm::vec3& _seg_end) const
{
    glm::vec3 segment_direction_vec = _seg_end - _seg_start;
    LEti::Math::shrink_vector_to_1(segment_direction_vec);

    glm::vec3 point_to_start_vec = _point - _seg_start;

    float dot = LEti::Math::dot_product(point_to_start_vec, segment_direction_vec);

    if(dot < 0.0f || dot > 1.0f)
        return -1.0f;

    return fabs(LEti::Math::dot_product(segment_direction_vec, point_to_start_vec));
}


SAT_Models_Intersection::Intersection_Data SAT_Models_Intersection::M_polygons_collision(const Polygon &_first, const Polygon &_second) const
{
	SAT_Models_Intersection::Intersection_Data result;

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


float SAT_Models_Intersection::M_smallest_point_to_polygon_distance(const glm::vec3 &_point, const Polygon &_pol) const
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

LDS::List<glm::vec3> SAT_Models_Intersection::M_points_of_contact(const Polygon_Holder_Base* _f_pols, unsigned int _f_count, const Polygon_Holder_Base* _s_pols, unsigned int _s_count) const
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


//LEti::Geometry::Simple_Intersection_Data SAT_Models_Intersection::intersection__polygon_vs_point(const Polygon& _polygon, const glm::vec3& _point) const
//{
//    LEti::Geometry_2D::Equasion_Data AB_eq(_polygon[0], _polygon[1]);
//    LEti::Geometry_2D::Equasion_Data BC_eq(_polygon[1], _polygon[2]);
//    LEti::Geometry_2D::Equasion_Data CA_eq(_polygon[2], _polygon[0]);

//	float AB_y_proj = AB_eq.solve_by_x(_point.x);
//	float BC_y_proj = BC_eq.solve_by_x(_point.x);
//	float CA_y_proj = CA_eq.solve_by_x(_point.x);

//	bool AB_right_side = AB_eq.is_vertical() ? ( _polygon[1].y < _polygon[0].y ? _point.x >= _polygon[0].x : _point.x <= _polygon[0].x ) : ( AB_eq.goes_left() ? AB_y_proj > _point.y : AB_y_proj < _point.y );
//	bool BC_right_side = BC_eq.is_vertical() ? ( _polygon[2].y < _polygon[1].y ? _point.x >= _polygon[1].x : _point.x <= _polygon[1].x ) : ( BC_eq.goes_left() ? BC_y_proj > _point.y : BC_y_proj < _point.y );
//	bool CA_right_side = CA_eq.is_vertical() ? ( _polygon[0].y < _polygon[2].y ? _point.x >= _polygon[2].x : _point.x <= _polygon[2].x ) : ( CA_eq.goes_left() ? CA_y_proj > _point.y : CA_y_proj < _point.y );

//	if (AB_right_side && BC_right_side && CA_right_side)
//        return LEti::Geometry::Simple_Intersection_Data(LEti::Geometry::Simple_Intersection_Data::Type::intersection, _point);
//    return LEti::Geometry::Simple_Intersection_Data();
//}



//LEti::Geometry::Simple_Intersection_Data SAT_Models_Intersection::collision__model_vs_point(const Physical_Model_2D &_model, const glm::vec3 &_point) const
//{
//	for (unsigned int i = 0; i < _model.get_polygons_count(); ++i)
//	{
//        LEti::Geometry::Simple_Intersection_Data id = intersection__polygon_vs_point(*_model.get_polygon(i), _point);
//		if(id)
//			return id;
//	}
//    return LEti::Geometry::Simple_Intersection_Data();
//}


LPhys::Intersection_Data SAT_Models_Intersection::collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _pols_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _pols_amount_2) const
{
    LPhys::Intersection_Data result(LPhys::Intersection_Data::Type::intersection);

    unsigned int first_collided_polygon = 0;
    unsigned int second_collided_polygon = 0;

    SAT_Models_Intersection::Intersection_Data f_id;
	for(unsigned int i=0; i<_pols_amount_1; ++i)
	{
		for(unsigned int j=0; j<_pols_amount_2; ++j)
		{
            SAT_Models_Intersection::Intersection_Data id = M_polygons_collision(*_polygon_holder_1->get_polygon(i), *_polygon_holder_2->get_polygon(j));

			if(!id.intersection)
				continue;

            if(!(f_id.min_dist < 0.0f || f_id.min_dist > id.min_dist))
                continue;

            f_id = id;
            first_collided_polygon = i;
            second_collided_polygon = j;
		}
	}

	if(!f_id.intersection)
        return {};

    result.normal = -f_id.min_dist_axis;
	LEti::Math::shrink_vector_to_1(result.normal);
    result.depth = f_id.min_dist;

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






























































