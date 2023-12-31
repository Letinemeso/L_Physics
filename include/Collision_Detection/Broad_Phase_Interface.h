#ifndef __BROAD_PHASE_INTERFACE
#define __BROAD_PHASE_INTERFACE

#include <Data_Structures/List.h>

#include <Modules/Physics_Module_2D.h>


namespace LPhys
{

	class Broad_Phase_Interface
	{
	public:
        using objects_list = LDS::List<const Physics_Module_2D*>;
        using points_list = LDS::List<const glm::vec3*>;

		struct Colliding_Pair
		{
            const Physics_Module_2D* first = nullptr, * second = nullptr;
            Colliding_Pair(const Physics_Module_2D* _first, const Physics_Module_2D* _second) : first(_first), second(_second) { L_ASSERT(!(first == second));}
			bool operator==(const Colliding_Pair& _other) const { return (first == _other.first && second == _other.second) || (first == _other.second && second == _other.first); }
			bool operator<(const Colliding_Pair& _other) const
			{
                const Physics_Module_2D* f_bigger = first > second ? first : second;
                const Physics_Module_2D* f_lesser = first > second ? second : first;
                const Physics_Module_2D* s_bigger = _other.first > _other.second ? _other.first : _other.second;
                const Physics_Module_2D* s_lesser = _other.first > _other.second ? _other.second : _other.first;
				return f_bigger < s_bigger ? true : f_lesser < s_lesser ? true : false;
			}
            bool operator>(const Colliding_Pair& _other) const { return !(*this < _other) && !(*this == _other); }
		};
        using Colliding_Pair_List = LDS::List<Colliding_Pair>;

		struct Colliding_Point_And_Object
		{
            const Physics_Module_2D* object = nullptr;
			const glm::vec3* point = nullptr;
            Colliding_Point_And_Object(const Physics_Module_2D* _object, const glm::vec3* _point) : object(_object), point(_point) {  }
			bool operator==(const Colliding_Point_And_Object& _other) const { return object == _other.object && point == _other.point; }
			bool operator<(const Colliding_Point_And_Object& _other) const { return object < _other.object; }
			bool operator>(const Colliding_Point_And_Object& _other) const { return !(*this < _other); }
		};
        using Colliding_Point_And_Object_List = LDS::List<Colliding_Point_And_Object>;

    protected:
        LDS::List<Colliding_Pair> m_possible_collisions__models;
        LDS::List<Colliding_Point_And_Object> m_possible_collisions__points;

	public:
		virtual ~Broad_Phase_Interface();

	public:
		virtual void update(const objects_list& _registred_objects, const points_list& _registred_points) = 0;

    public:
        inline const LDS::List<Colliding_Pair>& possible_collisions__models() const { return m_possible_collisions__models; }
        inline const LDS::List<Colliding_Point_And_Object>& possible_collisions__points() const { return m_possible_collisions__points; }

	};

}

#endif // __BROAD_PHASE_INTERFACE
