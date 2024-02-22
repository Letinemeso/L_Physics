#include <Physical_Models/Border.h>

using namespace LPhys;


Border::Border()
{

}

Border::Border(const Border& _other)
{
    m_offset = _other.m_offset;
    m_size = _other.m_size;
}

void Border::operator=(const Border& _other)
{
    m_offset = _other.m_offset;
    m_size = _other.m_size;
}



bool Border::M_is_valid() const
{
    for(unsigned int i=0; i<3; ++i)
    {
        if(m_size[i] < 0.0f)
            return false;
    }
    return true;
}



Border& Border::consider_point(const glm::vec3 &_point)
{
    if(!M_is_valid())
    {
        m_offset = _point;
        m_size = {0.0f, 0.0f, 0.0f};
        return *this;
    }

    for(unsigned int i=0; i<3; ++i)
    {
        if(_point[i] < m_offset[i])
        {
            m_size[i] += m_offset[i] - _point[i];
            m_offset[i] = _point[i];
        }
        else if(_point[i] > m_offset[i] + m_size[i])
        {
            m_size[i] = _point[i] - m_offset[i];
        }
    }

    return *this;
}



bool Border::point_is_inside(const glm::vec3& _point) const
{
    for(unsigned int i=0; i<3; ++i)
    {
        if(_point[i] < m_offset[i] - 0.0001f)
            return false;
        if(_point[i] > m_offset[i] + m_size[i] + 0.0001f)
            return false;
    }

    return true;
}



Border Border::operator&&(const Border &_other) const
{
    if(!M_is_valid() || !_other.M_is_valid())
        return {};

    Border result;

    for(unsigned int i=0; i<3; ++i)
    {
        result.m_offset[i] = m_offset[i] > _other.m_offset[i] ? m_offset[i] : _other.m_offset[i];
        float this_opposite = m_offset[i] + m_size[i];
        float other_opposite = _other.m_offset[i] + _other.m_size[i];
        result.m_size[i] = (this_opposite < other_opposite ? this_opposite : other_opposite) - result.m_offset[i];

        if(result.m_size[i] < 0.0f)
            return {};
    }

    return result;
}

Border Border::operator||(const Border &_other) const
{
    if(!M_is_valid())
        return _other;
    if(!_other.M_is_valid())
        return *this;

    Border result = *this;

    result.consider_point(_other.m_offset);
    result.consider_point(_other.m_offset + _other.m_size);

    return result;
}

bool Border::operator==(const Border &_other) const
{
    for(unsigned int i=0; i<3; ++i)
    {
        if(!LEti::Math::floats_are_equal(m_offset[i], _other.m_offset[i]))
            return false;
        if(!LEti::Math::floats_are_equal(m_size[i], _other.m_size[i]))
            return false;
    }
    return true;
}

Border::operator bool() const
{
    return M_is_valid();
}
