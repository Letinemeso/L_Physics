#pragma once

#include <vec3.hpp>
#include <mat4x4.hpp>

#include <L_Debug/L_Debug.h>
#include <Data_Structures/Vector.h>
#include <Stuff/Border.h>

#include <Stuff/Math_Stuff.h>

#include <Physical_Models/Polygon.h>


namespace LPhys
{

    class Polygon_Holder_Base
    {
    private:
        unsigned int m_amount = 0;

    public:
        Polygon_Holder_Base() { }
        virtual ~Polygon_Holder_Base() { }

    public:
        inline unsigned int amount() const { return m_amount; }

    public:
        virtual Polygon_Holder_Base* create_copy() const = 0;
        virtual void allocate(unsigned int _amount) { m_amount = _amount; };
        virtual Polygon* get_polygon(unsigned int _index) = 0;
        virtual const Polygon* get_polygon(unsigned int _index) const = 0;

    };

    template<typename T>
    class Polygon_Holder final : public Polygon_Holder_Base
    {
    private:
        T* polygons = nullptr;

    public:
        inline Polygon_Holder_Base* create_copy() const override { return new Polygon_Holder<T>; }
        inline void allocate(unsigned int _amount) override { Polygon_Holder_Base::allocate(_amount); delete[] polygons; polygons = new T[_amount]; }
        inline Polygon* get_polygon(unsigned int _index) override { L_ASSERT(_index < amount()); return &polygons[_index]; }
        inline const Polygon* get_polygon(unsigned int _index) const override { L_ASSERT(_index < amount()); return &polygons[_index]; }

    public:
        ~Polygon_Holder() { delete[] polygons; }

    };


    class Physical_Model_Imprint;

    class Physical_Model
    {
	private:
        LDS::Vector<float> m_raw_coords;
        LDS::Vector<bool> m_collision_permissions;

        Polygon_Holder_Base* m_polygons_holder = nullptr;

        bool m_cache_polygons_borders = false;

    private:
        LST::Border m_border;
        LDS::Vector<LST::Border> m_polygons_borders_cache;

    private:
        virtual Polygon_Holder_Base* M_create_polygons_holder() const;

	private:
        void M_init_border();
        void M_update_border(const glm::mat4x4& _matrix);
        void M_update_polygons_borders_if_enabled(const glm::mat4x4& _matrix);

	public:
        inline const LST::Border& border() const { return m_border; }
        inline const LDS::Vector<float>& raw_coords() const { return m_raw_coords; }

	public:
        Physical_Model();
        virtual ~Physical_Model();

    public:
        inline void set_cache_polygons_borders(bool _value) { m_cache_polygons_borders = _value; }

    public:
        void setup(const LDS::Vector<float>& _raw_coords, const LDS::Vector<bool>& _collision_permissions);

        virtual void update(const glm::mat4x4& _matrix);

    public:
        inline const Polygon_Holder_Base* get_polygons() const { return m_polygons_holder; }
        inline bool caching_polygons_borders() const {  return m_cache_polygons_borders; }
        inline const LDS::Vector<LST::Border>& polygons_borders() const { return m_polygons_borders_cache; }

        Physical_Model_Imprint* create_imprint() const;

	};

    class Physical_Model_Imprint
    {
    private:
        const Physical_Model* m_parent = nullptr;

    private:
        Polygon_Holder_Base* m_polygons_holder = nullptr;
        LST::Border m_border;
        LDS::Vector<LST::Border> m_polygons_borders_cache;

    public:
        Physical_Model_Imprint(const Physical_Model* _parent);
        Physical_Model_Imprint(Physical_Model_Imprint&& _other);
        Physical_Model_Imprint(const Physical_Model_Imprint& _other);
        ~Physical_Model_Imprint();

    private:
        void M_update_border(const glm::mat4x4& _matrix);

    public:
        void update(const glm::mat4x4& _translation, const glm::mat4x4& _rotation, const glm::mat4x4& _scale);
        void update_with_single_matrix(const glm::mat4x4& _matrix);
        void update_to_current_model_state();

        inline const Physical_Model* get_parent() const { return m_parent; }
        inline const Polygon_Holder_Base* get_polygons() const { return m_polygons_holder; }
        inline const LST::Border& border() const { return m_border; }
        inline const LDS::Vector<LST::Border>& polygons_borders() const { return m_polygons_borders_cache; }

    };
}
