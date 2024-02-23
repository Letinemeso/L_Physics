#pragma once

#include <Data_Structures/List.h>

#include <Modules/Physics_Module.h>


namespace LPhys
{

	class Broad_Phase_Interface
	{
	public:
        using Objects_List = LDS::List<const Physics_Module*>;

		struct Colliding_Pair
		{
            const Physics_Module* first = nullptr, * second = nullptr;
            Colliding_Pair(const Physics_Module* _first, const Physics_Module* _second) : first(_first), second(_second) { L_ASSERT(!(first == second));}
			bool operator==(const Colliding_Pair& _other) const { return (first == _other.first && second == _other.second) || (first == _other.second && second == _other.first); }
			bool operator<(const Colliding_Pair& _other) const
			{
                const Physics_Module* f_bigger = first > second ? first : second;
                const Physics_Module* f_lesser = first > second ? second : first;
                const Physics_Module* s_bigger = _other.first > _other.second ? _other.first : _other.second;
                const Physics_Module* s_lesser = _other.first > _other.second ? _other.second : _other.first;
				return f_bigger < s_bigger ? true : f_lesser < s_lesser ? true : false;
			}
            bool operator>(const Colliding_Pair& _other) const { return !(*this < _other) && !(*this == _other); }
		};
        using Colliding_Pair_List = LDS::List<Colliding_Pair>;

    protected:
        LDS::List<Colliding_Pair> m_possible_collisions__models;

	public:
		virtual ~Broad_Phase_Interface();

	public:
        virtual void update(const Objects_List& _registred_objects) = 0;

    public:
        inline const LDS::List<Colliding_Pair>& possible_collisions__models() const { return m_possible_collisions__models; }

	};

}
