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

bool check_with_index(const glm::vec3& _point, const Polygon& _polygon, unsigned int _index, const Border& _polygon_border, const Line* _polygon_lines, float _tolerance)
{
    Line vertex_to_opposite_side_line(_polygon[_index], _point);

    Lines_Intersection_Data opposite_intersection = vertex_to_opposite_side_line.calculate_intersection_with(_polygon_lines[(_index + 1) % 3], _tolerance);

    if(!opposite_intersection.intersection)
        return false;

    for(unsigned int i=0; i<3; ++i)
    {
        if(!value_is_between(_point[i], _polygon[_index][i], opposite_intersection.point[i]))
            return false;
    }

    return true;
}


bool LPhys::point_is_inside_polygon(const glm::vec3& _point, const Polygon& _polygon, float _tolerance)
{
    Border polygon_border;
    for(unsigned int i=0; i<3; ++i)
        polygon_border.consider_point(_polygon[i]);

    Line polygon_lines[3];

    for(unsigned int i=0; i<3; ++i)
        polygon_lines[i].init(_polygon[i], _polygon[i + 1]);

    if(!polygon_border.point_is_inside(_point))
        return false;

    for(unsigned int i=0; i<3; ++i)
    {
        if(!check_with_index(_point, _polygon, i, polygon_border, polygon_lines, _tolerance))
            return false;
    }

    return true;
}
