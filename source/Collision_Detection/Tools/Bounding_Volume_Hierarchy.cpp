#include <Collision_Detection/Tools/Bounding_Volume_Hierarchy.h>

using namespace LPhys;


namespace LPhys
{

    struct Polygon_Data
    {
        const Polygon* polygon = nullptr;
        unsigned int id = 0;
    };

    using Polygons_Vec = LDS::Vector<Polygon_Data>;

    using Exclusions_Vec = LDS::Vector<bool>;

    unsigned int calculate_exclusions_amount(unsigned int _elements_amount)
    {
        if(_elements_amount < 2)
            return 0;

        return ( _elements_amount * ( _elements_amount - 1 ) ) / 2;
    }

    unsigned int calculate_exclusion_index(unsigned int _first_index, unsigned int _second_index, unsigned int _total_amount)
    {
        L_ASSERT(_first_index != _second_index);

        if(_first_index > _second_index)
            std::swap(_first_index, _second_index);

        return (_first_index * _total_amount) - calculate_exclusions_amount(_first_index + 1) + (_second_index - _first_index - 1);
    }

    struct Polygons_In_Area
    {
        Border area;
        Polygons_Vec polygons_1;
        Polygons_Vec polygons_2;
    };

    Polygons_In_Area find_polygons_in_area(const Polygons_Vec& _p1, const Polygons_Vec& _p2, const Border& _border)
    {
        Polygons_In_Area result;
        result.polygons_1.resize(_p1.size());
        result.polygons_2.resize(_p2.size());

        for(unsigned int p_i = 0; p_i < _p1.size(); ++p_i)
        {
            const Polygon_Data& data = _p1[p_i];
            const Polygon& polygon = *data.polygon;

            bool inside = false;
            for(unsigned int i = 0; i < 3; ++i)
            {
                if( _border.point_is_inside(polygon[i]) )
                {
                    inside = true;
                    break;
                }
            }

            if(!inside)
                continue;

            result.polygons_1.push(data);

            for(unsigned int i = 0; i < 3; ++i)
                result.area.consider_point(polygon[i]);
        }

        for(unsigned int p_i = 0; p_i < _p2.size(); ++p_i)
        {
            const Polygon_Data& data = _p2[p_i];
            const Polygon& polygon = *data.polygon;

            bool inside = false;
            for(unsigned int i = 0; i < 3; ++i)
            {
                if( _border.point_is_inside(polygon[i]) )
                {
                    inside = true;
                    break;
                }
            }

            if(!inside)
                continue;

            result.polygons_2.push(data);

            for(unsigned int i = 0; i < 3; ++i)
                result.area.consider_point(polygon[i]);
        }

        return result;
    }

    bool same_polygons(const Polygons_Vec& _v1, const Polygons_Vec& _v2)
    {
        if(_v1.size() != _v2.size())
            return false;

        for(unsigned int i = 0; i < _v1.size(); ++i)
        {
            if(_v1[i].id != _v2[i].id)
                return false;
        }

        return true;
    }

    void append_polygons_pairs(Possible_Colliding_Polygons& _result,
                               Exclusions_Vec& _exclusions,
                               unsigned int _total_polygons_amount,
                               const Polygons_Vec& _p1, const
                               Polygons_Vec& _p2)
    {
        for(unsigned int i_1 = 0; i_1 < _p1.size(); ++i_1)
        {
            const Polygon_Data& data_1 = _p1[i_1];
            for(unsigned int i_2 = 0; i_2 < _p2.size(); ++i_2)
            {
                const Polygon_Data& data_2 = _p2[i_2];

                unsigned int exclusion_index = calculate_exclusion_index(data_1.id, data_2.id, _total_polygons_amount);
                if(_exclusions[exclusion_index])
                    continue;

                _result.push({data_1.polygon, data_2.polygon});
                _exclusions[exclusion_index] = true;
            }
        }
    }

    void find_possible_colliding_polygons_in_area(Possible_Colliding_Polygons& _result,
                                                  Exclusions_Vec& _exclusions,
                                                  const Polygons_Vec& _p1,
                                                  const Polygons_Vec& _p2,
                                                  const Border& _border,
                                                  unsigned int _total_polygons_amount,
                                                  unsigned int _min_polygons,
                                                  unsigned int _repeated_depth_limit)
    {
        if(_p1.size() == 0 || _p2.size() == 0)
            return;

        if(_repeated_depth_limit >= 3)
            return append_polygons_pairs(_result, _exclusions, _total_polygons_amount, _p1, _p2);

        if(_p1.size() < _min_polygons && _p2.size() < _min_polygons)
            return append_polygons_pairs(_result, _exclusions, _total_polygons_amount, _p1, _p2);

        Border b1 = _border;
        Border b2 = _border;

        if(_border.size().x > _border.size().y && _border.size().x > _border.size().z)
        {
            glm::vec3 diff = {_border.size().x * 0.5f, 0.0f, 0.0f};

            b1.modify_size(-diff);
            b2.modify_size(-diff);
            b2.modify_offset(diff);
        }
        else if(_border.size().y > _border.size().z)
        {
            glm::vec3 diff = {0.0f, _border.size().y * 0.5f, 0.0f};

            b1.modify_size(-diff);
            b2.modify_size(-diff);
            b2.modify_offset(diff);
        }
        else
        {
            glm::vec3 diff = {0.0f, 0.0f, _border.size().z * 0.5f};

            b1.modify_size(-diff);
            b2.modify_size(-diff);
            b2.modify_offset(diff);
        }

        Polygons_In_Area p1 = find_polygons_in_area(_p1, _p2, b1);
        Polygons_In_Area p2 = find_polygons_in_area(_p1, _p2, b2);

        unsigned int counter_1 = 0;
        if(same_polygons(p1.polygons_1, _p1) && same_polygons(p1.polygons_2, _p2))
            counter_1 = _repeated_depth_limit + 1;

        unsigned int counter_2 = 0;
        if(same_polygons(p2.polygons_1, _p1) && same_polygons(p2.polygons_2, _p2))
            counter_2 = _repeated_depth_limit + 1;

        if(same_polygons(p1.polygons_1, p2.polygons_1) && same_polygons(p1.polygons_2, p2.polygons_2))
        {
            find_possible_colliding_polygons_in_area(_result, _exclusions, p1.polygons_1, p1.polygons_2, p1.area, _total_polygons_amount, _min_polygons, counter_1);
            return;
        }

        find_possible_colliding_polygons_in_area(_result, _exclusions, p1.polygons_1, p1.polygons_2, p1.area, _total_polygons_amount, _min_polygons, counter_1);
        find_possible_colliding_polygons_in_area(_result, _exclusions, p2.polygons_1, p2.polygons_2, p2.area, _total_polygons_amount, _min_polygons, counter_2);
    }


    void init_polygons_vec(Polygons_Vec& _vec,
                                Border& _common_border,
                                const Polygon_Holder_Base& _polygons,
                                const LDS::Vector<Border>& _borders_cache,
                                const Border& _initial_area,
                                unsigned int _id_offset)
    {
        if(_borders_cache.size() > 0)
        {
            L_ASSERT(_polygons.amount() == _borders_cache.size());

            for(unsigned int p_i = 0; p_i < _polygons.amount(); ++p_i)
            {
                const Polygon& polygon = *_polygons.get_polygon(p_i);
                const Border& polygon_border = _borders_cache[p_i];

                if(!polygon_border.intersects_with(_initial_area))
                    continue;

                _vec.push({&polygon, _vec.size() + _id_offset});
                _common_border.expand_with(polygon_border);
            }
        }
        else
        {
            for(unsigned int p_i = 0; p_i < _polygons.amount(); ++p_i)
            {
                const Polygon& polygon = *_polygons.get_polygon(p_i);
                Border polygon_border = polygon.construct_border();

                if(!polygon_border.intersects_with(_initial_area))
                    continue;

                _vec.push({&polygon, _vec.size() + _id_offset});
                _common_border.expand_with(polygon_border);
            }
        }
    }

}


Possible_Colliding_Polygons LPhys::find_possible_colliding_polygons(const Polygon_Holder_Base& _polygons_1,
                                                                    const Polygon_Holder_Base& _polygons_2,
                                                                    unsigned int _min_polygons_in_area)
{
    Polygons_Vec polygons_1(_polygons_1.amount());
    Polygons_Vec polygons_2(_polygons_2.amount());

    Border common_area;

    for(unsigned int p_i = 0; p_i < _polygons_1.amount(); ++p_i)
    {
        const Polygon& polygon = *_polygons_1.get_polygon(p_i);
        polygons_1.push({&polygon, p_i});

        for(unsigned int i = 0; i < 3; ++i)
            common_area.consider_point(polygon[i]);
    }
    for(unsigned int p_i = 0; p_i < _polygons_2.amount(); ++p_i)
    {
        const Polygon& polygon = *_polygons_2.get_polygon(p_i);
        polygons_2.push({&polygon, p_i + _polygons_1.amount()});

        for(unsigned int i = 0; i < 3; ++i)
            common_area.consider_point(polygon[i]);
    }

    unsigned int total_polygon_amount = polygons_1.size() + polygons_2.size();
    unsigned int exclusions_size = calculate_exclusions_amount(total_polygon_amount);
    LDS::Vector<bool> exclusions(exclusions_size, false);

    Possible_Colliding_Polygons result( (polygons_1.size() > polygons_2.size() ? polygons_1.size() : polygons_2.size()) / 2 );

    find_possible_colliding_polygons_in_area(result, exclusions, polygons_1, polygons_2, common_area, total_polygon_amount, _min_polygons_in_area, 0);

    return result;
}

Possible_Colliding_Polygons LPhys::find_possible_colliding_polygons(const Polygon_Holder_Base& _polygons_1,
                                                                    const LDS::Vector<Border>& _borders_cache_1,
                                                                    const Polygon_Holder_Base& _polygons_2,
                                                                    const LDS::Vector<Border>& _borders_cache_2,
                                                                    unsigned int _min_polygons_in_area,
                                                                    const Border& _initial_area)
{
    if(!_initial_area.valid())
        return {};

    Polygons_Vec polygons_1(_polygons_1.amount());
    Polygons_Vec polygons_2(_polygons_2.amount());

    Border common_area;

    init_polygons_vec(polygons_1, common_area, _polygons_1, _borders_cache_1, _initial_area, 0);
    if(polygons_1.size() == 0)
        return {};

    init_polygons_vec(polygons_2, common_area, _polygons_2, _borders_cache_2, _initial_area, polygons_1.size());
    if(polygons_2.size() == 0)
        return {};

    unsigned int total_polygon_amount = polygons_1.size() + polygons_2.size();
    unsigned int exclusions_size = calculate_exclusions_amount(total_polygon_amount);
    LDS::Vector<bool> exclusions(exclusions_size, false);

    Possible_Colliding_Polygons result( (polygons_1.size() > polygons_2.size() ? polygons_1.size() : polygons_2.size()) / 2 );

    find_possible_colliding_polygons_in_area(result, exclusions, polygons_1, polygons_2, common_area, total_polygon_amount, _min_polygons_in_area, 0);

    return result;
}
