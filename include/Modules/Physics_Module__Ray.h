#pragma once

#include <Modules/Physics_Module.h>

namespace LPhys
{

    class Physics_Module__Ray : public Physics_Module
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module__Ray, LPhys::Physics_Module);

    private:
        glm::vec3 m_base_start;
        glm::vec3 m_base_direction;

        glm::vec3 m_current_start;
        glm::vec3 m_current_direction;

    public:
        inline void set_start(const glm::vec3& _value) { m_base_start = _value; }
        inline void set_direction(const glm::vec3& _value) { m_base_direction = _value; }

        inline const glm::vec3& base_start() const { return m_base_start; }
        inline const glm::vec3& base_direction() const { return m_base_direction; }

        inline const glm::vec3& current_start() const { return m_current_start; }
        inline const glm::vec3& current_direction() const { return m_current_direction; }

    public:
        void expand_border(Border& _border) const override;
        bool may_intersect_with_other(const Physics_Module& _other) const override;
        bool intersects_with_border(const Border& _border) const override;

    public:
        void update(float _dt) override;

    };


    class Physics_Module_Stub__Ray : public Physics_Module_Stub
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module_Stub__Ray, LPhys::Physics_Module_Stub);

    public:
        INIT_FIELDS
        ADD_FIELD(glm::vec3, start)
        ADD_FIELD(glm::vec3, direction)
        FIELDS_END

    public:
        glm::vec3 start;
        glm::vec3 direction;

    public:
        INIT_BUILDER_STUB(Physics_Module__Ray);

    };

}
