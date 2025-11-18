#pragma once

#include <Stuff/Thread_Pool.h>

#include <Collision_Detection/Narrow_Phase/Narrow_Phase_Interface.h>
#include <Collision_Detection/Primitives/Primitives_Intersection_Detector.h>
#include <Modules/Physics_Module__Mesh.h>


namespace LPhys
{

    class Dynamic_Narrow_CD__Mobile_Vs_Static : public Narrow_Phase_Interface
    {
    private:
        unsigned int m_interpolation_precision = 3;
        unsigned int m_segments_division_precision = 7;

        float m_segment_piece_multiplier = 1.0f / (float)(m_segments_division_precision + 1);

    private:
        Primitives_Intersection_Detector* m_intersection_detector = nullptr;

        LST::Thread_Pool m_thread_pool;
        std::mutex m_save_intersection_mutex;

    public:
        Dynamic_Narrow_CD__Mobile_Vs_Static();
        ~Dynamic_Narrow_CD__Mobile_Vs_Static();

    public:
        inline void set_interpolation_precision(unsigned int _value) { m_interpolation_precision = _value; }
        inline void set_segments_division_precision(unsigned int _value) { m_segments_division_precision = _value; m_segment_piece_multiplier = 1.0f / (float)(_value + 1); }
        inline void set_intersection_detector(Primitives_Intersection_Detector* _detector) { delete m_intersection_detector; m_intersection_detector = _detector; }

    private:
        struct Ratio_Pair
        {
            float min = 2.0f;
            float max = -1.0f;

            inline void consider_value(float _value)
            {
                if((_value < 0.0f) || (_value > 1.0f))
                    return;

                if(min > _value)
                    min = _value;
                if(max < _value)
                    max = _value;
            }

            inline bool valid() const
            {
                return min <= max;
            }
        };

        struct Segment
        {
            glm::vec3 start;
            glm::vec3 end;

            inline glm::vec3 direction() const
            {
                return end - start;
            }
        };

        LDS::Vector<const Polygon*> M_find_possibly_colliding_polygons(const Border& _border, const Physics_Module__Mesh& _static) const;
        void M_construct_polygon_segments(LDS::Vector<Segment>& _segments, const Polygon& _prev, const Polygon& _curr) const;
        Ratio_Pair M_find_possible_collision_timeframe_for_segment(const Segment& _segment, const LDS::Vector<const Polygon*>& _polygons) const;
        Ratio_Pair M_find_possible_collision_timeframe(const Physics_Module__Mesh& _mobile, const Physics_Module__Mesh& _static) const;

    private:
        Intersection_Data M_get_precise_time_ratio_of_collision(const Physics_Module__Mesh& _mobile, const Physics_Module__Mesh& _static, float _min_ratio, float _max_ratio) const;

    public:
        Intersection_Data objects_collide(const Physics_Module__Mesh& _mobile, const Physics_Module__Mesh& _static) const;

    public:
        void update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions) override;

    };

}
