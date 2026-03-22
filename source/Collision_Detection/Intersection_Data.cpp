#include <Collision_Detection/Intersection_Data.h>

using namespace LPhys;


Intersection_Data::Intersection_Data()
{

}

Intersection_Data::Intersection_Data(const Intersection_Data& _other)
    : intersection(_other.intersection), points(_other.points), normal(_other.normal), depth(_other.depth),
    first(_other.first), second(_other.second),
    first_collided_polygon_index(_other.first_collided_polygon_index), second_collided_polygon_index(_other.second_collided_polygon_index),
    time_of_intersection_ratio(_other.time_of_intersection_ratio),
    time_of_intersection_ratio_step(_other.time_of_intersection_ratio_step)
{

}

Intersection_Data::Intersection_Data(Intersection_Data&& _other)
    : intersection(_other.intersection), normal(_other.normal), depth(_other.depth),
    first(_other.first), second(_other.second),
    first_collided_polygon_index(_other.first_collided_polygon_index), second_collided_polygon_index(_other.second_collided_polygon_index),
    time_of_intersection_ratio(_other.time_of_intersection_ratio),
    time_of_intersection_ratio_step(_other.time_of_intersection_ratio_step)
{
    points = LST::move(_other.points);
}

void Intersection_Data::operator=(const Intersection_Data& _other)
{
    intersection = _other.intersection;
    points = _other.points;
    time_of_intersection_ratio = _other.time_of_intersection_ratio;
    time_of_intersection_ratio_step = _other.time_of_intersection_ratio_step;
    normal = _other.normal;
    depth = _other.depth;
    first = _other.first;
    second = _other.second;
    first_collided_polygon_index = _other.first_collided_polygon_index;
    second_collided_polygon_index = _other.second_collided_polygon_index;
}
