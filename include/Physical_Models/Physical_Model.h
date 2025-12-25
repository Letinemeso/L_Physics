#pragma once

#include <vec3.hpp>
#include <mat4x4.hpp>

#include <L_Debug/L_Debug.h>
#include <Data_Structures/Vector.h>

#include <Math_Stuff.h>

#include <Physical_Models/Border.h>
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
		float* m_raw_coords = nullptr;
		unsigned int m_raw_coords_count = 0;

		bool* m_collision_permissions = nullptr;

        Polygon_Holder_Base* m_polygons_holder = nullptr;

        bool m_cache_polygons_borders = false;

    private:
        Border m_border;
        LDS::Vector<Border> m_polygons_borders_cache;

    private:
        virtual Polygon_Holder_Base* M_create_polygons_holder() const;

	private:
        void M_update_border();
        void M_update_polygons_borders_if_enabled();

	public:
        inline const Border& border() const { return m_border; }
        inline const float* raw_coords() const { return m_raw_coords; }
        inline unsigned int raw_coords_count() const { return m_raw_coords_count; }

	public:
        Physical_Model();
        Physical_Model(const Physical_Model& _other);

        virtual ~Physical_Model();

    public:
        inline void set_cache_polygons_borders(bool _value) { m_cache_polygons_borders = _value; }

    public:
		void setup(const float* _raw_coords, unsigned int _raw_coords_count, const bool* _collision_permissions);
        void move_raw(const glm::vec3& _stride);

        virtual void update(const glm::mat4x4& _matrix);
        void copy_real_coordinates(const Physical_Model& _other);

    public:
        inline const Polygon_Holder_Base* get_polygons() const { return m_polygons_holder; }
        inline bool caching_polygons_borders() const {  return m_cache_polygons_borders; }
        inline const LDS::Vector<Border>& polygons_borders() const { return m_polygons_borders_cache; }

        Physical_Model_Imprint* create_imprint() const;

	};

    class Physical_Model_Imprint
    {
    private:
        const Physical_Model* m_parent = nullptr;

    private:
        Polygon_Holder_Base* m_polygons_holder = nullptr;
        Border m_border;
        LDS::Vector<Border> m_polygons_borders_cache;

    public:
        Physical_Model_Imprint(const Physical_Model* _parent);
        Physical_Model_Imprint(Physical_Model_Imprint&& _other);
        Physical_Model_Imprint(const Physical_Model_Imprint& _other);
        ~Physical_Model_Imprint();

    private:
        void M_update_border();

    public:
        void update(const glm::mat4x4& _translation, const glm::mat4x4& _rotation, const glm::mat4x4& _scale);
        void update_with_single_matrix(const glm::mat4x4& _matrix);
        void update_to_current_model_state();

        inline const Physical_Model* get_parent() const { return m_parent; }
        inline const Polygon_Holder_Base* get_polygons() const { return m_polygons_holder; }
        inline const Border& border() const { return m_border; }
        inline const LDS::Vector<Border>& polygons_borders() const { return m_polygons_borders_cache; }

    };
}
