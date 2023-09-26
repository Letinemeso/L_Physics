#ifndef __SPACE_HASHER_2D
#define __SPACE_HASHER_2D

#include <L_Debug/L_Debug.h>
#include <Data_Structures/AVL_Tree.h>

#include <Collision_Detection/Broad_Phase_Interface.h>

namespace LPhys
{

	class Space_Hasher_2D : public Broad_Phase_Interface
	{
	private:
		unsigned int m_precision = 0;
		unsigned int m_array_size = 0;
		objects_list** m_array = nullptr;
		unsigned int m_number_binary_length = 0;
		struct Space_Border { float min_x = 0.0f, min_y = 0.0f, height = 0.0f, width = 0.0f; };
		Space_Border m_space_borders;

	private:
		unsigned int get_number_binary_length(unsigned int _number);
		unsigned int construct_hash(unsigned int _x, unsigned int _y);
		void update_border(const objects_list& _registred_objects);
		void reset_hash_array();
		void hash_objects(const objects_list& _registred_objects);
		void check_for_possible_collisions__points(const points_list &_registred_points);
		void check_for_possible_collisions__models();

	public:
		Space_Hasher_2D() { }
		virtual ~Space_Hasher_2D();

	public:
        void set_precision(unsigned int _precision);

        void update(const objects_list &_registred_objects, const points_list &_registred_points) override;

	};

}

#endif // __SPACE_HASHER_2D
