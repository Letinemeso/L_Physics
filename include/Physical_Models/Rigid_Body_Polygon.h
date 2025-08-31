#pragma once

#include <Physical_Models/Polygon.h>

namespace LPhys
{

    class Rigid_Body_Polygon : public Polygon
    {
    private:
        float m_mass = 0.0f;

    public:
        Rigid_Body_Polygon();
        Rigid_Body_Polygon(const Rigid_Body_Polygon& _other);
        ~Rigid_Body_Polygon();

        void setup(const Polygon& _other) override;

    public:
        inline void set_mass(float _mass) { m_mass = _mass; }

    public:
        inline float mass() const { return m_mass; }

    };

}
