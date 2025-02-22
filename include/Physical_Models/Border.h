#pragma once

#include <Math_Stuff.h>


namespace LPhys
{

    class Border
    {
    private:
        glm::vec3 m_offset;
        glm::vec3 m_size{-1.0f, -1.0f, -1.0f};
        bool m_valid = false;

    public:
        Border();
        Border(const Border& _other);
        void operator=(const Border& _other);

        inline void reset() { m_size = {-1.0f, -1.0f, -1.0f}; m_valid = false; }

        inline void set_offset(const glm::vec3& _offset) { m_offset = _offset; }
        inline void set_size(const glm::vec3& _size) { m_size = _size; M_update_validness(); }

        inline void modify_offset(const glm::vec3& _by) { m_offset += _by; }
        inline void modify_size(const glm::vec3& _by) { m_size += _by; }

    public:
        inline const glm::vec3& offset() const { return m_offset; }
        inline const glm::vec3& size() const { return m_size; }
        inline bool valid() const { return m_valid; }

        inline float left() const { return m_offset.x; }
        inline float right() const { return m_offset.x + m_size.x; }
        inline float bottom() const { return m_offset.y; }
        inline float top() const { return m_offset.y + m_size.y; }
        inline float back() const { return m_offset.z; }
        inline float front() const { return m_offset.z + m_size.z; }

    private:
        void M_update_validness();

    public:
        Border& consider_point(const glm::vec3& _point);
        Border& expand_with(const Border& _other);

    public:
        bool point_is_inside(const glm::vec3& _point) const;
        bool intersects_with(const Border& _other) const;

    public:
        Border operator&&(const Border& _other) const;
        Border operator||(const Border& _other) const;
        bool operator==(const Border& _other) const;
        inline operator bool() const { return m_valid; }

    };

}
