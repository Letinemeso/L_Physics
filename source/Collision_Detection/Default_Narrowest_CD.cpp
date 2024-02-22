#include <Collision_Detection/Default_Narrowest_CD.h>

using namespace LPhys;


Default_Narrowest_CD::Default_Narrowest_CD()
{

}

Default_Narrowest_CD::~Default_Narrowest_CD()
{

}



LEti::Geometry::Simple_Intersection_Data Default_Narrowest_CD::intersection__polygon_vs_point(const Polygon& _polygon, const glm::vec3& _point) const
{
    LEti::Geometry_2D::Equasion_Data AB_eq(_polygon[0], _polygon[1]);
    LEti::Geometry_2D::Equasion_Data BC_eq(_polygon[1], _polygon[2]);
    LEti::Geometry_2D::Equasion_Data CA_eq(_polygon[2], _polygon[0]);

    float AB_y_proj = AB_eq.solve_by_x(_point.x);
    float BC_y_proj = BC_eq.solve_by_x(_point.x);
    float CA_y_proj = CA_eq.solve_by_x(_point.x);

    bool AB_right_side = AB_eq.is_vertical() ? ( _polygon[1].y < _polygon[0].y ? _point.x >= _polygon[0].x : _point.x <= _polygon[0].x ) : ( AB_eq.goes_left() ? AB_y_proj > _point.y : AB_y_proj < _point.y );
    bool BC_right_side = BC_eq.is_vertical() ? ( _polygon[2].y < _polygon[1].y ? _point.x >= _polygon[1].x : _point.x <= _polygon[1].x ) : ( BC_eq.goes_left() ? BC_y_proj > _point.y : BC_y_proj < _point.y );
    bool CA_right_side = CA_eq.is_vertical() ? ( _polygon[0].y < _polygon[2].y ? _point.x >= _polygon[2].x : _point.x <= _polygon[2].x ) : ( CA_eq.goes_left() ? CA_y_proj > _point.y : CA_y_proj < _point.y );

    if (AB_right_side && BC_right_side && CA_right_side)
        return LEti::Geometry::Simple_Intersection_Data(LEti::Geometry::Simple_Intersection_Data::Type::intersection, _point);
    return LEti::Geometry::Simple_Intersection_Data();
}


Intersection_Data Default_Narrowest_CD::intersection__polygon_vs_segment(const Polygon& _polygon, const LEti::Geometry::Segment& _segment) const
{
    unsigned int found_intersections = 0;

    Intersection_Data result_id;
    LEti::Geometry::Simple_Intersection_Data ids[3];

    ids[0] = LEti::Geometry_2D::segments_intersect({_polygon[0], _polygon[1]}, _segment);
    ids[1] = LEti::Geometry_2D::segments_intersect({_polygon[1], _polygon[2]}, _segment);
    ids[2] = LEti::Geometry_2D::segments_intersect({_polygon[2], _polygon[0]}, _segment);
    for(unsigned int i=0; i<3; ++i)
    {
        if(ids[i])
        {
            ++found_intersections;
            std::pair<glm::vec3, glm::vec3> normals = LEti::Geometry::get_segments_normals(ids[i].first, ids[i].second);
            result_id.normal += normals.first;
            result_id.normal -= normals.second;
            result_id.point += ids[i].point;
        }
    }

    if(found_intersections == 0)
        return Intersection_Data();

    result_id.type = Intersection_Data::Type::intersection;
    LEti::Math::shrink_vector_to_1(result_id.normal);
    result_id.point /= (float)found_intersections;

    return result_id;
}

Intersection_Data Default_Narrowest_CD::intersection__polygon_vs_polygon(const Polygon& _first, const Polygon& _second) const
{
    Border this_rb;
    this_rb.consider_point(_first[0]).consider_point(_first[1]).consider_point(_first[2]);
    Border other_rb;
    other_rb.consider_point(_second[0]).consider_point(_second[1]).consider_point(_second[2]);

    if(!(this_rb && other_rb))
        return Intersection_Data();

    unsigned int found_intersections = 0;
    Intersection_Data result_id;
    Intersection_Data ids[3];

    ids[0] = intersection__polygon_vs_segment(_first, {_second[0], _second[1]});
    ids[1] = intersection__polygon_vs_segment(_first, {_second[1], _second[2]});
    ids[2] = intersection__polygon_vs_segment(_first, {_second[2], _second[0]});
    for(unsigned int i=0; i<3; ++i)
    {
        if(ids[i])
        {
            ++found_intersections;
            result_id.normal += ids[i].normal;
            result_id.point += ids[i].point;
        }
    }

    if(found_intersections != 0)
    {
        result_id.type = Intersection_Data::Type::intersection;
        LEti::Math::shrink_vector_to_1(result_id.normal);
        result_id.point /= (float)found_intersections;

        return result_id;
    }

    LEti::Geometry::Simple_Intersection_Data _3 = intersection__polygon_vs_point(_first, _second[0]);
    LEti::Geometry::Simple_Intersection_Data _4 = intersection__polygon_vs_point(_first, _second[1]);
    LEti::Geometry::Simple_Intersection_Data _5 = intersection__polygon_vs_point(_first, _second[2]);
    if(_3 && _4 && _5)
        return Intersection_Data(Intersection_Data::Type::intersection, glm::vec3(_3+_4+_5)/3.0f);
    else
    {L_ASSERT(!(_3 || _4 || _5));}
    return Intersection_Data();
}

Intersection_Data Default_Narrowest_CD::collision__model_vs_segment(const Physical_Model_2D& _model, const LEti::Geometry::Segment& _segment) const
{
    Intersection_Data result_id;
    unsigned int found_intersections = 0;

    for (unsigned int i = 0; i < _model.get_polygons_count(); ++i)
    {
        Intersection_Data id = intersection__polygon_vs_segment(*_model.get_polygon(i), _segment);
        if (id)
        {
            ++found_intersections;
            result_id.normal += id.normal;
            result_id.point += id.point;
        }
    }

    if(found_intersections == 0)
        return Intersection_Data(Intersection_Data::Type::none);

    result_id.type = Intersection_Data::Type::intersection;
    result_id.point /= (float)found_intersections;
    LEti::Math::shrink_vector_to_1(result_id.normal);

    return result_id;
}



LEti::Geometry::Simple_Intersection_Data Default_Narrowest_CD::collision__model_vs_point(const Physical_Model_2D& _model, const glm::vec3& _point) const
{
    for (unsigned int i = 0; i < _model.get_polygons_count(); ++i)
    {
        LEti::Geometry::Simple_Intersection_Data id = intersection__polygon_vs_point(*_model.get_polygon(i), _point);
        if(id)
            return id;
    }
    return LEti::Geometry::Simple_Intersection_Data();
}


Intersection_Data Default_Narrowest_CD::collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _pols_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _pols_amount_2) const
{
    Intersection_Data result_id;
    unsigned int found_intersections = 0;

    for (unsigned int i = 0; i < _pols_amount_1; ++i)
    {
        for (unsigned int j = 0; j < _pols_amount_2; ++j)
        {
            Intersection_Data id = intersection__polygon_vs_polygon(*_polygon_holder_1->get_polygon(i), *_polygon_holder_2->get_polygon(j));
            if (id)
            {
                ++found_intersections;
                result_id.normal += id.normal;
                result_id.point += id.point;
            }
        }
    }

    if(found_intersections != 0)
    {
        result_id.type = Intersection_Data::Type::intersection;
        result_id.point /= (float)found_intersections;

        if(LEti::Math::vector_length(result_id.normal) < 0.0001f) return Intersection_Data();

        LEti::Math::shrink_vector_to_1(result_id.normal);

        return result_id;
    }

    for (unsigned int i = 0; i < _pols_amount_2; ++i)
    {
        for (unsigned int j = 0; j < _pols_amount_1; ++j)
        {
            Intersection_Data id = intersection__polygon_vs_polygon(*_polygon_holder_1->get_polygon(i), *_polygon_holder_2->get_polygon(j));
            if (id)
            {
                ++found_intersections;
                result_id.normal += id.normal;
                result_id.point += id.point;
            }
        }
    }

    if(found_intersections != 0)
    {
        result_id.type = Intersection_Data::Type::intersection;
        result_id.point /= (float)found_intersections;

        if(LEti::Math::vector_length(result_id.normal) < 0.0001f) return Intersection_Data();

        LEti::Math::shrink_vector_to_1(result_id.normal);

        return result_id;
    }

    return Intersection_Data();
}
