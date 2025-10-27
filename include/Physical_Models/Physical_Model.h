#pragma once

#include "vec3.hpp"
#include "mat4x4.hpp"

#include "L_Debug/L_Debug.h"
#include "Data_Structures/List.h"

#include <Math_Stuff.h>

#include <Physical_Models/Border.h>
#include <Physical_Models/Polygon.h>


namespace LPhys
{

    class Polygon_Holder_Base
    {
    public:
        Polygon_Holder_Base() { }
        virtual ~Polygon_Holder_Base() { }

    public:
        virtual Polygon_Holder_Base* create_copy() const = 0;
        virtual void allocate(unsigned int _amount) = 0;
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
        inline void allocate(unsigned int _amount) override { delete[] polygons;  polygons = new T[_amount]; }
        inline Polygon* get_polygon(unsigned int _index) override { return &polygons[_index]; }
        inline const Polygon* get_polygon(unsigned int _index) const override { return &polygons[_index]; }

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
		unsigned int m_polygons_count = 0;

    private:
        glm::vec3 m_center_of_mass{0.0f, 0.0f, 0.0f};

	private:
        Border m_border;

    private:
        virtual Polygon_Holder_Base* M_create_polygons_holder() const;

	private:
        void M_update_border();
        virtual glm::vec3 M_calculate_center_of_mass() const;

	public:
        inline const Border& border() const { return m_border; }
        inline const float* raw_coords() const { return m_raw_coords; }
        inline unsigned int raw_coords_count() const { return m_raw_coords_count; }

	public:
        Physical_Model();
        Physical_Model(const Physical_Model& _other);
		void setup(const float* _raw_coords, unsigned int _raw_coords_count, const bool* _collision_permissions);
		void move_raw(const glm::vec3& _stride);

        virtual ~Physical_Model();

        virtual void update(const glm::mat4x4& _matrix);
        void copy_real_coordinates(const Physical_Model& _other);

        Physical_Model_Imprint* create_imprint() const;

	public:
        const Polygon* get_polygon(unsigned int _index) const;
        const Polygon_Holder_Base* get_polygons() const;
        unsigned int get_polygons_count() const;

        inline const glm::vec3& center_of_mass() const { return m_center_of_mass; }

	};

    class Physical_Model_Imprint
    {
    private:
        const Physical_Model* m_parent = nullptr;

    private:
        Polygon_Holder_Base* m_polygons_holder = nullptr;
        unsigned int m_polygons_count = 0;
        Border m_border;

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

        const Physical_Model* get_parent() const;
        const Polygon* get_polygon(unsigned int _index) const;
        const Polygon_Holder_Base* get_polygons() const;
        unsigned int get_polygons_count() const;
        const Border& border() const;

    };
}
