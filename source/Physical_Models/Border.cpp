#include <Physical_Models/Border.h>

#include <L_Debug/L_Debug.h>

using namespace LPhys;


Border::Border()
{

}

Border::Border(const Border& _other)
{
    m_offset = _other.m_offset;
    m_size = _other.m_size;
    m_valid = _other.m_valid;
}

void Border::operator=(const Border& _other)
{
    m_offset = _other.m_offset;
    m_size = _other.m_size;
    m_valid = _other.m_valid;
}



void Border::M_update_validness()
{
    for(unsigned int i=0; i<3; ++i)
    {
        if(m_size[i] < 0.0f)
        {
            m_valid = false;
            return;
        }
    }
    m_valid = true;
}



Border& Border::consider_point(const glm::vec3 &_point)
{
    L_DEBUG_FUNC_NOARG([&]()
    {
        for(unsigned int i = 0; i < 3; ++i)
            L_ASSERT(!std::isnan(_point[i]));
    });

    if(!m_valid)
    {
        m_offset = _point;
        m_size = {0.0f, 0.0f, 0.0f};
        m_valid = true;
        return *this;
    }

    for(unsigned int i = 0; i < 3; ++i)
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

Border& Border::expand_with(const Border& _other)
{
    if(!m_valid && !_other.m_valid)
        return *this;

    if(!m_valid)
    {
        m_offset = _other.m_offset;
        m_size = _other.m_size;
        M_update_validness();
        return *this;
    }

    if(!_other.m_valid)
        return *this;

    consider_point(_other.m_offset);
    consider_point(_other.m_offset + _other.m_size);

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

bool Border::intersects_with(const Border& _other) const
{
    if(!m_valid || !_other.m_valid)
        return false;

    for(unsigned int i=0; i<3; ++i)
    {
        float check_offset = m_offset[i] > _other.m_offset[i] ? m_offset[i] : _other.m_offset[i];
        float this_opposite = m_offset[i] + m_size[i];
        float other_opposite = _other.m_offset[i] + _other.m_size[i];
        float check_size = (this_opposite < other_opposite ? this_opposite : other_opposite) - check_offset;

        if(check_size < 0.0f)
            return false;
    }

    return true;
}



Border Border::operator&&(const Border &_other) const
{
    if(!m_valid || !_other.m_valid)
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

    result.m_valid = true;
    return result;
}

Border Border::operator||(const Border &_other) const
{
    if(!m_valid)
        return _other;
    if(!_other.m_valid)
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
        if(!LST::Math::floats_are_equal(m_offset[i], _other.m_offset[i]))
            return false;
        if(!LST::Math::floats_are_equal(m_size[i], _other.m_size[i]))
            return false;
    }
    return true;
}
