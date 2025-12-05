#include <Collision_Detection/Intersection_Data.h>

using namespace LPhys;


Intersection_Data::Intersection_Data()
{

}

Intersection_Data::Intersection_Data(bool _intersection)
    : intersection(_intersection)
{

}

Intersection_Data::Intersection_Data(bool _intersection, const glm::vec3& _point)
    : intersection(_intersection), point(_point)
{

}

Intersection_Data::Intersection_Data(const Intersection_Data& _other)
    : intersection(_other.intersection), point(_other.point), normal(_other.normal), depth(_other.depth),
    first(_other.first), second(_other.second),
    first_collided_polygon_index(_other.first_collided_polygon_index), second_collided_polygon_index(_other.second_collided_polygon_index),
    time_of_intersection_ratio(_other.time_of_intersection_ratio),
    time_of_intersection_ratio_step(_other.time_of_intersection_ratio_step)
{

}

Intersection_Data::Intersection_Data(Intersection_Data&& _other)
    : intersection(_other.intersection), point(_other.point), normal(_other.normal), depth(_other.depth),
    first(_other.first), second(_other.second),
    first_collided_polygon_index(_other.first_collided_polygon_index), second_collided_polygon_index(_other.second_collided_polygon_index),
    time_of_intersection_ratio(_other.time_of_intersection_ratio),
    time_of_intersection_ratio_step(_other.time_of_intersection_ratio_step)
{

}

void Intersection_Data::operator=(const Intersection_Data& _other)
{
    intersection = _other.intersection;
    point = _other.point;
    time_of_intersection_ratio = _other.time_of_intersection_ratio;
    time_of_intersection_ratio_step = _other.time_of_intersection_ratio_step;
    normal = _other.normal;
    depth = _other.depth;
    first = _other.first;
    second = _other.second;
    first_collided_polygon_index = _other.first_collided_polygon_index;
    second_collided_polygon_index = _other.second_collided_polygon_index;
}
