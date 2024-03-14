#include <Collision_Detection/Primitives/Polygon_Vs_Point_Intersection.h>

using namespace LPhys;

bool value_is_between(float _value, float _min, float _max)
{
    if(_min > _max)
    {
        float temp = _min;
        _min = _max;
        _max = temp;
    }

    _min -= 0.0001f;
    _max += 0.0001f;

    return _value >= _min && _value <= _max;
}


bool LPhys::point_is_inside_polygon(const glm::vec3& _point, const Polygon& _polygon)
{
    Border polygon_border;
    for(unsigned int i=0; i<3; ++i)
        polygon_border.consider_point(_polygon[i]);

    Line polygon_lines[3];

    for(unsigned int i=0; i<3; ++i)
    {
        polygon_lines[i].init(_polygon[i], _polygon[i + 1]);

        if(!polygon_lines[i].contains_point(_point))
            continue;

        if(polygon_border.point_is_inside(_point))
            return true;
    }

    glm::vec3 certain_intersection = _polygon[0] + ( (_polygon[1] - _polygon[0]) * 0.5f );

    Line point_to_edge_line(_point, certain_intersection);

    Lines_Intersection_Data second_intersection = point_to_edge_line.calculate_intersection_with(polygon_lines[1]);
    if(!second_intersection || !polygon_border.point_is_inside(second_intersection.point))
        second_intersection = point_to_edge_line.calculate_intersection_with(polygon_lines[2]);

    if(!second_intersection || !polygon_border.point_is_inside(second_intersection.point))
        return false;

    for(unsigned int i=0; i<3; ++i)
    {
        if(!value_is_between(_point[i], certain_intersection[i], second_intersection.point[i]))
            return false;
    }

    return true;
}
