#pragma once

#include <Stuff/Thread_Pool.h>

#include <Collision_Detection/Narrow_Phase/Narrow_Phase_Interface.h>
#include <Collision_Detection/Primitives/Primitives_Intersection_Detector.h>
#include <Modules/Physics_Module__Mesh.h>


namespace LPhys
{

    class Dynamic_Narrow_CD : public Narrow_Phase_Interface
	{
    private:
		unsigned int m_precision = 10;

    private:
        Primitives_Intersection_Detector* m_intersection_detector = nullptr;

        LST::Thread_Pool m_thread_pool;
        std::mutex m_save_intersection_mutex;

    public:
        Dynamic_Narrow_CD();
        ~Dynamic_Narrow_CD();

    public:
        inline void set_precision(unsigned int _precision) { m_precision = _precision; }
        inline void set_intersection_detector(Primitives_Intersection_Detector* _detector) { delete m_intersection_detector; m_intersection_detector = _detector; }

	private:
        struct Ratio_Pair
        {
            float min = 0.0f;
            float max = 1.0f;

            inline void consider_value(float _value)
            {
                if((_value < 0.0f) || (_value > 1.0f))
                    return;

                if(min < _value)
                    min = _value;
                if(max > _value)
                    max = _value;
            }
        };
        Ratio_Pair M_find_possible_collision_timeframe(const Physics_Module__Mesh& _first, const Physics_Module__Mesh& _second) const;

	private:
        Intersection_Data get_precise_time_ratio_of_collision(const Physics_Module__Mesh& _first, const Physics_Module__Mesh& _second, float _min_ratio, float _max_ratio) const;

	public:
        Intersection_Data objects_collide(const Physics_Module__Mesh& _first, const Physics_Module__Mesh& _second) const;

	public:
        void update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions) override;

	};

}
