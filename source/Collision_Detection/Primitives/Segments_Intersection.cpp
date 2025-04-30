#include <Collision_Detection/Primitives/Segments_Intersection.h>

#include <L_Debug/L_Debug.h>

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

bool point_is_between(const glm::vec3& _point, const glm::vec3& _min, const glm::vec3& _max)
{
    glm::vec3 min_to_max = _max - _min;
    glm::vec3 min_to_point = _point - _min;

    for(unsigned int i=0; i<3; ++i)
    {
        if( min_to_max[i] < 0.0f != min_to_point[i] < 0.0f )
            return false;
        if( fabs(min_to_max[i]) < fabs(min_to_point[i]) )
            return false;
    }

    return true;
}

Lines_Intersection_Data calculate_non_parallel_intersection(const glm::vec3& _first_start, const glm::vec3& _first_end, const glm::vec3& _second_start, const glm::vec3& _second_end, const Lines_Intersection_Data& _lines_intersection_data)
{
    for(unsigned int i=0; i<3; ++i)
    {
        if(!value_is_between(_lines_intersection_data.point[i], _first_start[i], _first_end[i]))
            return {};
        if(!value_is_between(_lines_intersection_data.point[i], _second_start[i], _second_end[i]))
            return {};
    }

    return _lines_intersection_data;
}

Lines_Intersection_Data calculate_parallel_intersection(const glm::vec3& _first_start, const glm::vec3& _first_end, const glm::vec3& _second_start, const glm::vec3& _second_end, Lines_Intersection_Data _lines_intersection_data)
{
    glm::vec3 point(0.0f, 0.0f, 0.0f);

    bool first_start_between = point_is_between(_first_start, _second_start, _second_end);
    bool first_end_between = point_is_between(_first_end, _second_start, _second_end);
    bool second_start_between = point_is_between(_second_start, _first_start, _first_end);
    bool second_end_between = point_is_between(_second_end, _first_start, _first_end);

    if( (first_start_between != first_end_between) && (second_start_between != second_end_between) )
        _lines_intersection_data.point = (( first_start_between ? _first_start : _first_end ) + ( second_start_between ? _second_start : _second_end )) * 0.5f;
    else if( first_start_between && first_end_between && second_start_between && second_end_between )
        _lines_intersection_data.point = (_first_start + _first_end) * 0.5f;
    else if( first_start_between && first_end_between )
        _lines_intersection_data.point = (_first_start + _first_end) * 0.5f;
    else if( second_start_between && second_end_between )
        _lines_intersection_data.point = (_second_start + _second_end) * 0.5f;
    else
    {
        L_ASSERT(false);    //  something is wrong in point_in_between
    }

    return _lines_intersection_data;
}



Lines_Intersection_Data LPhys::calculate_segments_intersection(const glm::vec3& _first_start, const glm::vec3& _first_end, const glm::vec3& _second_start, const glm::vec3& _second_end)
{
    Line line_1(_first_start, _first_end);
    Line line_2(_second_start, _second_end);

    Lines_Intersection_Data lines_intersection = line_1.calculate_intersection_with(line_2);

    switch(lines_intersection.intersection)
    {
    case Lines_Intersection_Data::Intersection_Type::Intersection:
        return calculate_non_parallel_intersection(_first_start, _first_end, _second_start, _second_end, lines_intersection);
    case Lines_Intersection_Data::Intersection_Type::Same_Line:
        return calculate_parallel_intersection(_first_start, _first_end, _second_start, _second_end, lines_intersection);
    default:
        return lines_intersection;
    }
}
