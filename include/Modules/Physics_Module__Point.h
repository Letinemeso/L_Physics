#pragma once

#include <Modules/Physics_Module.h>


namespace LPhys
{

    class Physics_Module__Point : public Physics_Module
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module__Point, LPhys::Physics_Module);

    private:
        glm::vec3 m_raw_point{0.0f, 0.0f, 0.0f};
        glm::vec3 m_current_point;

    public:
        void set_point(const glm::vec3& _point);

    public:
        const glm::vec3& point() const { return m_current_point; }

    public:
        bool may_intersect_with_other(const Physics_Module& _other) const override;
        bool intersects_with_border(const Border& _border) const override;

    public:
        void update(float _dt) override;

    };


    class Physics_Module_Stub__Point : public Physics_Module_Stub
    {
    public:
        INIT_VARIABLE(LPhys::Physics_Module_Stub__Point, LPhys::Physics_Module_Stub);

        INIT_FIELDS
        ADD_FIELD(glm::vec3, point)
        FIELDS_END

    public:
        glm::vec3 point{0.0f, 0.0f, 0.0f};

    public:
        INIT_BUILDER_STUB(Physics_Module__Point)

    };

}
