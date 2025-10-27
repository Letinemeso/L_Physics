#pragma once

#include <Physical_Models/Physical_Model.h>
#include <Physical_Models/Rigid_Body_Polygon.h>

namespace LPhys
{

    class Rigid_Body_Physical_Model : public Physical_Model
    {
    private:
        float* m_masses = nullptr;
        float m_total_mass = 0.0f;
        float m_moment_of_inertia = 0.0f;

    public:
        Rigid_Body_Physical_Model();
        Rigid_Body_Physical_Model(const Rigid_Body_Physical_Model& _other);
        ~Rigid_Body_Physical_Model();

    private:
        Polygon_Holder_Base* M_create_polygons_holder() const override;

    private:
        glm::vec3 M_calculate_center_of_mass() const override;
        float M_calculate_moment_of_inertia() const;

    public:
        void update(const glm::mat4x4& _matrix) override;

    public:
        void set_masses(const float* _masses);

    public:
        inline float total_mass() const { return m_total_mass; }
        inline float moment_of_inertia() const { return m_moment_of_inertia; }

    };

}
