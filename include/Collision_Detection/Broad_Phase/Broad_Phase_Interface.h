#pragma once

#include <Data_Structures/Vector.h>
#include <Stuff/Function_Wrapper.h>

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
        LDS::List<Colliding_Pair> m_possible_collisions;

    public:
        using Filter_Function = LST::Function<bool(const Physics_Module*, const Physics_Module*)>;
        using Filters_List = LDS::List<Filter_Function>;

    private:
        Filters_List m_filters;

	public:
		virtual ~Broad_Phase_Interface();

    public:
        inline void reset_filters() { m_filters.clear(); }
        inline void add_filter(const Filter_Function& _filter) { m_filters.push_back(_filter); }

    public:
        bool passes_filters(const Physics_Module* _first, const Physics_Module* _second) const;

	public:
        virtual void reset() = 0;
        virtual void add_models(const Objects_List& _objects) = 0;
        virtual void process() = 0;

    public:
        inline const Colliding_Pair_List& possible_collisions() const { return m_possible_collisions; }

	};

}
