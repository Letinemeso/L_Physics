#ifndef __DYNAMIC_NARROW_CD
#define __DYNAMIC_NARROW_CD

#include <Collision_Detection/Narrow_Phase_Interface.h>


namespace LPhys
{

    class Dynamic_Narrow_CD : public Narrow_Phase_Interface
	{
    private:
		unsigned int m_precision = 10;

    public:
        inline void set_precision(unsigned int _precision) { m_precision = _precision; }

	private:
		using float_pair = std::pair<float, float>;
        float_pair find_ratio(const Physics_Module_2D& _moving_1, const Physics_Module_2D& _moving_2) const;

	private:
        Intersection_Data get_precise_time_ratio_of_collision(const Physics_Module_2D& _first, const Physics_Module_2D& _second, float _min_ratio, float _max_ratio, const Narrowest_Phase_Interface* _cd) const;
        Intersection_Data collision__moving_vs_moving(const Physics_Module_2D& _moving_1, const Physics_Module_2D& _moving_2, const Narrowest_Phase_Interface* _cd) const;
        Intersection_Data collision__moving_vs_static(const Physics_Module_2D& _moving, const Physics_Module_2D& _static, const Narrowest_Phase_Interface* _cd) const;

        LEti::Geometry::Simple_Intersection_Data collision__static_vs_point(const Physics_Module_2D& _static, const glm::vec3& _point, const Narrowest_Phase_Interface* _cd) const;

	public:
        Intersection_Data objects_collide(const Physics_Module_2D& _first, const Physics_Module_2D& _second, const Narrowest_Phase_Interface* _cd) const;

	public:
        void update(const Broad_Phase_Interface::Colliding_Pair_List& _possible_collisions__models, const Broad_Phase_Interface::Colliding_Point_And_Object_List& _possible_collisions__points, const Narrowest_Phase_Interface* _cd) override;

	};

}


#endif // __DYNAMIC_NARROW_CD
