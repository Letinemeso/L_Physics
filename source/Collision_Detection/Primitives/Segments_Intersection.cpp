#include <Collision_Detection/Primitives/Segments_Intersection.h>

using namespace LPhys;


Lines_Intersection_Data LPhys::calculate_segments_intersection(const glm::vec3& _first_start, const glm::vec3& _first_end, const glm::vec3& _second_start, const glm::vec3& _second_end)
{
    Line line_1(_first_start, _first_end);
    Line line_2(_second_start, _second_end);

    Lines_Intersection_Data lines_intersection = line_1.calculate_intersection_with(line_2);

    if(!lines_intersection.has_intersection)
        return lines_intersection;

    auto value_is_between = [](float _value, float _min, float _max)->bool
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
    };

    for(unsigned int i=0; i<3; ++i)
    {
        if(!value_is_between(lines_intersection.point[i], _first_start[i], _first_end[i]))
            return {};
        if(!value_is_between(lines_intersection.point[i], _second_start[i], _second_end[i]))
            return {};
    }

    return lines_intersection;
}
