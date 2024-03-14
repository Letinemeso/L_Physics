#include <Collision_Detection/Primitives/Lines_Intersection.h>

using namespace LPhys;


Line::Line()
{

}

Line::Line(const glm::vec3& _point_1, const glm::vec3& _point_2)
{
    init(_point_1, _point_2);
}



void Line::init(const glm::vec3& _point_1, const glm::vec3& _point_2)
{
    m_direction = _point_2 - _point_1;
    m_initial_offset = _point_1;
}



float Line::M_calculate_multiplier_by_component(Components _component, float _value) const
{
    L_ASSERT(can_be_solved_by_component(_component));

    return (_value - m_initial_offset[_component]) / m_direction[_component];
}

unsigned int Line::M_get_easy_solution_component(const Line& _with) const
{
    for(unsigned int i=0; i<3; ++i)
    {
        if(can_be_solved_by_component(i) && !_with.can_be_solved_by_component(i))
            return i;
    }
    return 3;
}

bool Line::M_matches_with(const Line& _with) const
{
    glm::vec3 this_direction = m_direction;
    glm::vec3 others_direction = _with.m_direction;

    LEti::Math::shrink_vector_to_1(this_direction);
    LEti::Math::shrink_vector_to_1(others_direction);

    for(unsigned int i=0; i<3; ++i)
    {
        if(!LEti::Math::floats_are_equal(fabs(this_direction[i]), fabs(others_direction[i])))
            return false;
    }

    return contains_point(_with.m_initial_offset);
}



bool Line::contains_point(const glm::vec3& _point, float _tolerance) const
{
    bool found_multiplier = false;
    float common_multiplier = 0.0f;

    for(unsigned int i=0; i<3; ++i)
    {
        if(!can_be_solved_by_component(i))
        {
            if(!LEti::Math::floats_are_equal(m_initial_offset[i], _point[i], _tolerance))
                return false;
            continue;
        }

        float multiplier = M_calculate_multiplier_by_component((Components)i, _point[i]);

        if(found_multiplier && !LEti::Math::floats_are_equal(common_multiplier, multiplier, _tolerance))
            return false;

        found_multiplier = true;
        common_multiplier = multiplier;
    }

    return true;
}

bool Line::can_be_solved_by_component(Components _component) const
{
    L_ASSERT(_component < Components::amount);

    return !LEti::Math::floats_are_equal(m_direction[_component], 0.0f, 0.000000001f);
}

glm::vec3 Line::solve_by_component(Components _component, float _value) const
{
    float multiplier = M_calculate_multiplier_by_component(_component, _value);

    return solve_by_multiplier(multiplier);
}

glm::vec3 Line::solve_by_multiplier(float _multiplier) const
{
    glm::vec3 result;

    for(unsigned int i=0; i<3; ++i)
        result[i] = m_direction[i] * _multiplier + m_initial_offset[i];

    return result;
}

Lines_Intersection_Data Line::calculate_intersection_with(const Line& _with) const
{
    unsigned int easy_solution_component = M_get_easy_solution_component(_with);

    if(easy_solution_component < 3)
    {
        float multiplier = (_with.m_initial_offset[easy_solution_component] - m_initial_offset[easy_solution_component]) / m_direction[easy_solution_component];
        glm::vec3 maybe_result_point = solve_by_multiplier(multiplier);
        if(_with.contains_point(maybe_result_point))
            return { Lines_Intersection_Data::Intersection, maybe_result_point };
        return {};
    }

    easy_solution_component = _with.M_get_easy_solution_component(*this);

    if(easy_solution_component < 3)
    {
        float multiplier = (m_initial_offset[easy_solution_component] - _with.m_initial_offset[easy_solution_component]) / _with.m_direction[easy_solution_component];
        glm::vec3 maybe_result_point = _with.solve_by_multiplier(multiplier);
        if(contains_point(maybe_result_point))
            return { Lines_Intersection_Data::Intersection, maybe_result_point };
        return {};
    }




    float comp_mult = 0.0f;
    float comp_offs = 0.0f;

    unsigned int first_multiplier_expression_component_index = 3;
    for(unsigned int i=0; i<3; ++i)
    {
        if(!can_be_solved_by_component(i))
            continue;

        comp_mult = _with.m_direction[i] / m_direction[i];
        comp_offs = (m_initial_offset[i] - _with.m_initial_offset[i]) / m_direction[i];
        first_multiplier_expression_component_index = i;
        break;
    }

    if(first_multiplier_expression_component_index == 3)
        return {};

    for(unsigned int i=0; i<3; ++i)
    {
        if(i == first_multiplier_expression_component_index)
            continue;

        comp_mult *= m_direction[i];
        comp_offs = comp_offs * m_direction[i] + m_initial_offset[i];

        comp_mult -= _with.m_direction[i];
        comp_offs = _with.m_initial_offset[i] - comp_offs;

        if(!LEti::Math::floats_are_equal(comp_mult, 0.0f))
            break;
    }

    if(LEti::Math::floats_are_equal(comp_mult, 0.0f, 0.0000001f))
    {
        if(M_matches_with(_with))
            return { Lines_Intersection_Data::Same_Line, m_initial_offset };

        return {};
    }

    float second_multiplier = comp_offs / comp_mult;

    glm::vec3 intersection_point = _with.solve_by_multiplier(second_multiplier);

    return { Lines_Intersection_Data::Intersection, intersection_point };
}

/*Lines_Intersection_Data Line::calculate_intersection_with(const Line& _with) const
{
    glm::vec3 line_between_starts = _with.m_initial_offset - m_initial_offset;

    glm::vec3 cross = LEti::Math::cross_product(m_direction, _with.m_direction);

    if(LEti::Math::floats_are_equal(LEti::Math::vector_length(cross), 0.0f))
        return {};

    float multiplier_first = (line_between_starts.x * cross.x + line_between_starts.y * cross.y + line_between_starts.z * cross.z) /
                             (cross.x * cross.x + cross.y * cross.y + cross.z * cross.z);

    float multiplier_second = (m_direction.x != 0) ? (line_between_starts.x + multiplier_first * m_direction.x) / _with.m_direction.x :
                                                     (m_direction.y != 0) ? (line_between_starts.y + multiplier_first * m_direction.y) / _with.m_direction.y :
                                                                            (line_between_starts.z + multiplier_first * m_direction.z) / _with.m_direction.z;

    Lines_Intersection_Data result;
    result.has_intersection = true;

    for(unsigned int i=0; i<3; ++i)
        result.point[i] = (m_direction[i] * multiplier_first) + m_initial_offset[i];

    return result;
}*/

/*Lines_Intersection_Data Line::calculate_intersection_with(const Line& _with) const
{
    float a1 = m_direction.x;
    float b1 = m_direction.y;
    float c1 = m_direction.z;
    float d1 = -a1 * m_initial_offset.x - b1 * m_initial_offset.y - c1 * m_initial_offset.z;

    // Вычисление параметров для прямой line2
    float a2 = _with.m_direction.x;
    float b2 = _with.m_direction.y;
    float c2 = _with.m_direction.z;
    float d2 = -a2 * _with.m_initial_offset.x - b2 * _with.m_initial_offset.y - c2 * _with.m_initial_offset.z;

    // Вычисление пересечения прямых
    float det = a1 * b2 - a2 * b1;
    if (det == 0) {
        std::cerr << "Прямые параллельны или совпадают, точка пересечения не существует.\n";
        return {}; // Возвращаем какой-то дефолтный результат
    } else {
        float x = (b1 * d2 - b2 * d1) / det;
        float y = (a2 * d1 - a1 * d2) / det;
        float z;
        if ( !LEti::Math::floats_are_equal(c1, 0.0f) )
            z = (-a1 * x - b1 * y - d1) / c1;
        else if ( !LEti::Math::floats_are_equal(c2, 0.0f) )
            z = (-a2 * x - b2 * y - d2) / c2;
        else
            z = 0.0f;

        return {true, {x, y, z}};
    }
}*/

/*Lines_Intersection_Data Line::calculate_intersection_with(const Line& _with) const
{
    float a1 = m_direction.x;               //  this stuff (and above) was provided by ChatGPT and does works bad. i was too lazy to implement it myself, but got a better idea for dependent part. i'll leave it like this for now
    float b1 = m_direction.y;
    float c1 = m_direction.z;
    float d1 = -a1 * m_initial_offset.x - b1 * m_initial_offset.y - c1 * m_initial_offset.z;

    float a2 = _with.m_direction.x;
    float b2 = _with.m_direction.y;
    float c2 = _with.m_direction.z;
    float d2 = -a2 * _with.m_initial_offset.x - b2 * _with.m_initial_offset.y - c2 * _with.m_initial_offset.z;

    float det = a1 * b2 - a2 * b1;
    if (det == 0)
        return {};

    float x = (b1 * d2 - b2 * d1) / det;
    if(m_direction.y * _with.m_direction.y < 0.0f)
        x = -x;

    float y = (a2 * d1 - a1 * d2) / det;
    if(m_direction.x * _with.m_direction.x < 0.0f)
        y = -y;

    float z = 0.0;
    if (c1 != 0 && c2 != 0) {
        z = (-a1 * x - b1 * y - d1) / c1;
    } else if (c1 == 0 && c2 != 0) {
        z = (-a2 * x - b2 * y - d2) / c2;
    }
    return {true, {x, y, z}};
}
*/
