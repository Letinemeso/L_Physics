#include <Collision_Detection/Primitives/SAT_Models_Intersection_3D.h>

#include <limits>

#include <Data_Structures/Vector.h>

#include <Collision_Detection/Primitives/Polygon_VS_Ray_Intersection.h>

using namespace LPhys;


SAT_Models_Intersection_3D::Polygons_Intersection_Data SAT_Models_Intersection_3D::M_calculate_intersection_point(const Polygon& _first, const Polygon& _second) const
{
    unsigned int intersections_amount = 0;
    glm::vec3 median_intersection_point = {0.0f, 0.0f, 0.0f};

    for(unsigned int i = 0; i < 3; ++i)
    {
        Polygon_VS_Ray_Intersection_Data id = segment_intersects_polygon(_first[i], _first[i + 1], _second);
        if(!id.intersection)
            continue;

        median_intersection_point += id.point;
        ++intersections_amount;
    }

    for(unsigned int i = 0; i < 3; ++i)
    {
        Polygon_VS_Ray_Intersection_Data id = segment_intersects_polygon(_second[i], _second[i + 1], _first);
        if(!id.intersection)
            continue;

        median_intersection_point += id.point;
        ++intersections_amount;
    }

    if(intersections_amount == 0)
        return {};

    Polygons_Intersection_Data result;
    result.intersection = true;
    result.point = median_intersection_point / (float)intersections_amount;

    return result;
}


SAT_Models_Intersection_3D::Axis_Intersection SAT_Models_Intersection_3D::M_check_intersection_on_axis(const glm::vec3& _axis, const Polygon& _first, const Polygon& _second) const
{
    MinMax_Pair first_pair;
    MinMax_Pair second_pair;

    for(unsigned int i = 0; i < 3; ++i)
    {
        float projection = LST::Math::dot_product(_first[i], _axis);
        first_pair.add_value(projection);
    }
    for(unsigned int i = 0; i < 3; ++i)
    {
        float projection = LST::Math::dot_product(_second[i], _axis);
        second_pair.add_value(projection);
    }

    if( !(first_pair.value_is_between(second_pair.min) || first_pair.value_is_between(second_pair.max) || (second_pair.value_is_between(first_pair.min) || second_pair.value_is_between(first_pair.max))) )
        return {};

    float dist_1 = first_pair.max - second_pair.min;
    float dist_2 = second_pair.max - first_pair.min;

    Axis_Intersection result;

    if(dist_1 <= 0.0f)
    {
        dist_1 = dist_2;
        result.flip_normal = true;
    }
    if(dist_2 > 0.0f && dist_1 > dist_2)
    {
        dist_1 = dist_2;
        result.flip_normal = true;
    }

    if(dist_1 <= 0.0f)
        return {};

    result.intersection = true;
    result.depth = dist_1;

    return result;
}

SAT_Models_Intersection_3D::Polygons_Intersection_Data SAT_Models_Intersection_3D::M_check_triangles_intersection(const Polygon& _first, const Polygon& _second) const
{
    glm::vec3 first_normal = LST::Math::cross_product(_first[2] - _first[1], _first[0] - _first[1]);
    glm::vec3 second_normal = LST::Math::cross_product(_second[2] - _second[1], _second[0] - _second[1]);

    float normals_dot = LST::Math::dot_product(first_normal, second_normal);
    if(normals_dot > 0.0f)
        return {};

    LST::Math::shrink_vector_to_1(first_normal);
    LST::Math::shrink_vector_to_1(second_normal);

    unsigned int plane_axes_amount = 2;
    glm::vec3 plane_axes[2];
    plane_axes[0] = first_normal;
    plane_axes[1] = second_normal;

    glm::vec3 edge_axes[9];
    unsigned int edge_axes_amount = 0;
    for(unsigned int f_i = 0; f_i < 3; ++f_i)
    {
        glm::vec3 f_edge = _first[f_i + 1] - _first[f_i];
        for(unsigned int s_i = 0; s_i < 3; ++s_i)
        {
            glm::vec3 s_edge = _second[s_i + 1] - _second[s_i];

            glm::vec3 axis = LST::Math::cross_product(f_edge, s_edge);
            if(LST::Math::vector_length_squared(axis) < 1e-9f)
                continue;

            LST::Math::shrink_vector_to_1(axis);
            edge_axes[edge_axes_amount] = axis;
            ++edge_axes_amount;
        }
    }

    Polygons_Intersection_Data plane_id;
    plane_id.depth = std::numeric_limits<float>::max();

    for(unsigned int i = 0; i < plane_axes_amount; ++i)
    {
        const glm::vec3& axis = plane_axes[i];

        Axis_Intersection id = M_check_intersection_on_axis(axis, _first, _second);
        if(!id.intersection)
            return plane_id;

        if(id.depth >= plane_id.depth)
            continue;

        plane_id.depth = id.depth;
        plane_id.normal = axis;
        if(id.flip_normal)
            plane_id.normal *= -1.0f;
    }

    Polygons_Intersection_Data edge_id;
    edge_id.depth = std::numeric_limits<float>::max();

    for(unsigned int i = 0; i < edge_axes_amount; ++i)
    {
        const glm::vec3& axis = edge_axes[i];

        Axis_Intersection id = M_check_intersection_on_axis(axis, _first, _second);
        if(!id.intersection)
            return edge_id;

        if(id.depth >= edge_id.depth)
            continue;

        edge_id.depth = id.depth;
        edge_id.normal = axis;
        if(id.flip_normal)
            edge_id.normal *= -1.0f;
    }

    Polygons_Intersection_Data result;

    float edge_depth_multiplied = edge_id.depth * m_plane_contact_priority_ratio;

    if(plane_id.depth < edge_depth_multiplied)
        result = plane_id;
    else
        result = edge_id;

    Polygons_Intersection_Data id = M_calculate_intersection_point(_first, _second);
    if(!id.intersection)
        return {};

    result.point = id.point;
    result.intersection = true;

    if(LST::Math::dot_product(result.normal, first_normal) < 0.0f)
        result.normal *= -1.0f;

    return result;
}


LDS::Vector<LST::Border> SAT_Models_Intersection_3D::M_construct_polygon_borders(const Polygon_Holder_Base* _polygon_holder) const
{
    LDS::Vector<LST::Border> result(_polygon_holder->amount());

    for(unsigned int i = 0; i < _polygon_holder->amount(); ++i)
        result.push( _polygon_holder->get_polygon(i)->construct_border() );

    return result;
}


SAT_Models_Intersection_3D::Common_Intersection_Data SAT_Models_Intersection_3D::M_calculate_common_intersection_simple(const Polygon_Holder_Base* _polygon_holder_1,
                                                       const LDS::Vector<LST::Border>& _polygons_borders_cache_1,
                                                       const Polygon_Holder_Base* _polygon_holder_2,
                                                       const LDS::Vector<LST::Border>& _polygons_borders_cache_2) const
{
    Common_Intersection_Data result;

    LST::Border border_cache_1;      //  crutch so lambda later can return reference instead of new object
    LST::Border border_cache_2;

    auto get_or_calculate_polygon_border = [](const LDS::Vector<LST::Border>& _polygons_borders_cache, const Polygon& _polygon, unsigned int _polygon_index, LST::Border& _cache)->const LST::Border&
    {
        if(_polygons_borders_cache.size() > 0)
            return _polygons_borders_cache[_polygon_index];

        _cache = _polygon.construct_border();
        return _cache;
    };

    for(unsigned int i_1 = 0; i_1 < _polygon_holder_1->amount(); ++i_1)
    {
        const Polygon& polygon_1 = *_polygon_holder_1->get_polygon(i_1);
        const LST::Border& border_1 = get_or_calculate_polygon_border(_polygons_borders_cache_1, polygon_1, i_1, border_cache_1);

        for(unsigned int i_2 = 0; i_2 < _polygon_holder_2->amount(); ++i_2)
        {
            const Polygon& polygon_2 = *_polygon_holder_2->get_polygon(i_2);
            const LST::Border& border_2 = get_or_calculate_polygon_border(_polygons_borders_cache_2, polygon_2, i_2, border_cache_2);

            if(!border_1.intersects_with(border_2))
                continue;

            Polygons_Intersection_Data id = M_check_triangles_intersection(polygon_1, polygon_2);

            if(!id.intersection)
                continue;

            result.points.push(id.point);
            result.normals.push(id.normal);
            result.depths.push(id.depth);
        }
    }

    return result;
}

SAT_Models_Intersection_3D::Common_Intersection_Data SAT_Models_Intersection_3D::M_calculate_common_intersection_bvh(const Polygon_Holder_Base* _polygon_holder_0, const LST::Border& _border_0,
                                                                                                                           const LDS::Vector<LST::Border>& _polygons_borders_cache_0,
                                                                                                                           const Polygon_Holder_Base* _polygon_holder_1, const LST::Border& _border_1,
                                                                                                                           const LDS::Vector<LST::Border>& _polygons_borders_cache_1) const
{
    Common_Intersection_Data result;

    LST::BVH::Context bvh_context;

    bvh_context.registered_borders.add(_polygons_borders_cache_0);
    bvh_context.registered_borders.add(_polygons_borders_cache_1);

    LST::Border common_border = _border_0 && _border_1;

    LST::BVH::ID_Pairs_Vec pairs = m_bvh.process(bvh_context, common_border);
    const LST::BVH::Registered_Borders& registered_borders = bvh_context.registered_borders;

    for(unsigned int i = 0; i < pairs.size(); ++i)
    {
        const LST::BVH::ID_Pair& pair = pairs[i];

        LST::BVH::Registered_Borders::Border_Location location_0 = registered_borders.calculate_border_location(pair.id_0);
        LST::BVH::Registered_Borders::Border_Location location_1 = registered_borders.calculate_border_location(pair.id_1);

        unsigned int p_0_index = location_0.index;
        unsigned int p_1_index = location_1.index;

        if(location_0.group > location_1.group)
            std::swap(p_0_index, p_1_index);

        const Polygon& p_0 = *_polygon_holder_0->get_polygon(p_0_index);
        const Polygon& p_1 = *_polygon_holder_1->get_polygon(p_1_index);

        Polygons_Intersection_Data id = M_check_triangles_intersection(p_0, p_1);

        if(!id.intersection)
            continue;

        result.points.push(id.point);
        result.normals.push(id.normal);
        result.depths.push(id.depth);
    }

    // m_bvh.reset_registered_borders();

    return result;
}

SAT_Models_Intersection_3D::Common_Intersection_Data SAT_Models_Intersection_3D::M_calculate_common_intersection(const Polygon_Holder_Base* _polygon_holder_1, const LST::Border& _border_1,
                                                                                                                 const LDS::Vector<LST::Border>& _polygons_borders_cache_1,
                                                                                                                 const Polygon_Holder_Base* _polygon_holder_2, const LST::Border& _border_2,
                                                                                                                 const LDS::Vector<LST::Border>& _polygons_borders_cache_2) const
{
    if(_polygon_holder_1->amount() < m_min_polygons_for_optimization && _polygon_holder_2->amount() < m_min_polygons_for_optimization)
    {
        return M_calculate_common_intersection_simple(_polygon_holder_1,
                                                      _polygons_borders_cache_1,
                                                      _polygon_holder_2,
                                                      _polygons_borders_cache_2);
    }
    else
    {
        return M_calculate_common_intersection_bvh(_polygon_holder_1, _border_1,
                                                   _polygons_borders_cache_1,
                                                   _polygon_holder_2, _border_2,
                                                   _polygons_borders_cache_2);
    }
}


void SAT_Models_Intersection_3D::M_calculate_combined_normal_and_depth(Intersection_Data& _result_id, const Common_Intersection_Data& _cid) const
{
    _result_id.normal = {0.0f, 0.0f, 0.0f};
    _result_id.depth = 0.0f;

    for(unsigned int i = 0; i < _cid.normals.size(); ++i)
    {
        glm::vec3 push_out_vector = -_cid.normals[i] * _cid.depths[i];

        _result_id.normal += push_out_vector;
        _result_id.depth += _cid.depths[i];
    }

    LST::Math::shrink_vector_to_1(_result_id.normal);
    _result_id.depth /= (float)_cid.depths.size();
}

LDS::Array<unsigned int, 4> SAT_Models_Intersection_3D::M_find_extreme_contacts_ids(const Common_Intersection_Data& _cid, const glm::vec3& _normal) const
{
    glm::vec3 center = {0.0f, 0.0f, 0.0f};
    for(unsigned int i = 0; i < _cid.points.size(); ++i)
        center += _cid.points[i];
    center /= (float)_cid.points.size();

    float extreme_0_min = std::numeric_limits<float>::max();
    float extreme_0_max = std::numeric_limits<float>::lowest();
    float extreme_1_min = std::numeric_limits<float>::max();
    float extreme_1_max = std::numeric_limits<float>::lowest();
    unsigned int extreme_0_min_index = LST::Math::Max_Unsigned_Int;
    unsigned int extreme_0_max_index = LST::Math::Max_Unsigned_Int;
    unsigned int extreme_1_min_index = LST::Math::Max_Unsigned_Int;
    unsigned int extreme_1_max_index = LST::Math::Max_Unsigned_Int;

    Basis basis(_normal);

    for(unsigned int i = 0; i < _cid.points.size(); ++i)
    {
        glm::vec3 point_w_offset = _cid.points[i] - center;

        float projection_0 = LST::Math::dot_product(point_w_offset, basis.axis_0);
        float projection_1 = LST::Math::dot_product(point_w_offset, basis.axis_1);

        if(projection_0 < extreme_0_min)
        {
            extreme_0_min = projection_0;
            extreme_0_min_index = i;
        }
        if(projection_0 > extreme_0_max)
        {
            extreme_0_max = projection_0;
            extreme_0_max_index = i;
        }

        if(projection_1 < extreme_1_min)
        {
            extreme_1_min = projection_1;
            extreme_1_min_index = i;
        }
        if(projection_1 > extreme_1_max)
        {
            extreme_1_max = projection_1;
            extreme_1_max_index = i;
        }
    }

    LDS::Array<unsigned int, 4> result;

    result.push(extreme_0_min_index);

    if(!result.contains(extreme_0_max_index))
        result.push(extreme_0_max_index);

    if(!result.contains(extreme_1_min_index))
        result.push(extreme_1_min_index);

    if(!result.contains(extreme_1_max_index))
        result.push(extreme_1_max_index);

    L_ASSERT(!result.contains(LST::Math::Max_Unsigned_Int));

    return result;
}

void SAT_Models_Intersection_3D::M_find_extreme_contacts(Intersection_Data& _result_id, const Common_Intersection_Data& _cid, const glm::vec3& _normal) const
{
    LDS::Array<unsigned int, 4> extreme_indices = M_find_extreme_contacts_ids(_cid, _normal);

    for(unsigned int i = 0; i < extreme_indices.size(); ++i)
    {
        unsigned int index = extreme_indices[i];

        _result_id.points.push( _cid.points[index] );

        glm::vec3 push_out_vec = -_cid.normals[index] * _cid.depths[index];
        float point_depth = LST::Math::dot_product(push_out_vec, _normal);
        if(point_depth < 0.0f)
            point_depth = 0.0f;

        _result_id.depths_per_point.push(point_depth);
    }
}



LPhys::Intersection_Data SAT_Models_Intersection_3D::collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_0,
                                                                               const LST::Border& _border_0,
                                                                               const LDS::Vector<LST::Border>& _polygons_borders_cache_0,
                                                                               const Polygon_Holder_Base* _polygon_holder_1,
                                                                               const LST::Border& _border_1,
                                                                               const LDS::Vector<LST::Border>& _polygons_borders_cache_1) const
{
    Common_Intersection_Data id = M_calculate_common_intersection(_polygon_holder_0, _border_0,
                                                                  _polygons_borders_cache_0.size() ? _polygons_borders_cache_0 : M_construct_polygon_borders(_polygon_holder_0),
                                                                  _polygon_holder_1, _border_1,
                                                                  _polygons_borders_cache_1.size() ? _polygons_borders_cache_1 : M_construct_polygon_borders(_polygon_holder_1));

    L_ASSERT(id.points.size() == id.normals.size());
    L_ASSERT(id.points.size() == id.depths.size());

    if(id.points.size() == 0)
        return {};

    Intersection_Data result;
    M_calculate_combined_normal_and_depth(result, id);

    if(result.depth < LST::Math::Float_Precision_Tolerance_Very_Risky)
        return {};

    result.intersection = true;
    M_find_extreme_contacts(result, id, result.normal);

    return result;
}
