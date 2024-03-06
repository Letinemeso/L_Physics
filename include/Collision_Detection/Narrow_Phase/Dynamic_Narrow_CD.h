#pragma once

#include <Collision_Detection/Narrow_Phase/Narrow_Phase_Interface.h>
#include <Collision_Detection/Primitives/SAT_Models_Intersection.h>


namespace LPhys
{

    class Dynamic_Narrow_CD : public Narrow_Phase_Interface
	{
    private:
		unsigned int m_precision = 10;

    private:
        SAT_Models_Intersection m_intersection_detector;

    public:
        inline void set_precision(unsigned int _precision) { m_precision = _precision; }

	private:
        struct Ratio_Pair
        {
            float min = 0.0f;
            float max = 1.0f;

            inline void consider_value(float _value)
            {
                if(_value < 0.0f || _value > 1.0f)
                    return;

                if(min < _value)
                    min = _value;
                if(max > _value)
                    max = _value;
            }
        };
        Ratio_Pair M_find_possible_collision_timeframe(const Physics_Module_2D& _first, const Physics_Module_2D& _second) const;

	private:
        Intersection_Data get_precise_time_ratio_of_collision(const Physics_Module_2D& _first, const Physics_Module_2D& _second, float _min_ratio, float _max_ratio) const;
        Intersection_Data collision__moving_vs_moving(const Physics_Module_2D& _moving_1, const Physics_Module_2D& _moving_2) const;

	public:
        Intersection_Data objects_collide(const Physics_Module_2D& _first, const Physics_Module_2D& _second) const;

	public:
        void update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions) override;

	};

}
