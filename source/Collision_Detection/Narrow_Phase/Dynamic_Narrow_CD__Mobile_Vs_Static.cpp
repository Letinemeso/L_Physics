#include <Collision_Detection/Narrow_Phase/Dynamic_Narrow_CD__Mobile_Vs_Static.h>

#include <Collision_Detection/Tools/Bounding_Volume_Hierarchy.h>
#include <Collision_Detection/Primitives/Polygon_VS_Ray_Intersection.h>

using namespace LPhys;


Dynamic_Narrow_CD__Mobile_Vs_Static::Dynamic_Narrow_CD__Mobile_Vs_Static()
{

}

Dynamic_Narrow_CD__Mobile_Vs_Static::~Dynamic_Narrow_CD__Mobile_Vs_Static()
{
    delete m_intersection_detector;
}



LDS::Vector<const Polygon*> Dynamic_Narrow_CD__Mobile_Vs_Static::M_find_possibly_colliding_polygons(const Border& _border, const Physics_Module__Mesh& _static) const
{
    const Polygon_Holder_Base& polygons = *_static.get_physical_model()->get_polygons();

    LDS::Vector<const Polygon*> result(polygons.amount());

    for(unsigned int p_i = 0; p_i < polygons.amount(); ++p_i)
    {
        const Polygon& polygon = *polygons.get_polygon(p_i);

        Border polygon_border;
        for(unsigned int i = 0; i < 3; ++i)
            polygon_border.consider_point(polygon[i]);

        if(_border.intersects_with(polygon_border))
            result.push(&polygon);
    }

    return result;
}

void Dynamic_Narrow_CD__Mobile_Vs_Static::M_construct_polygon_segments(LDS::Vector<Segment>& _segments, const Polygon& _prev, const Polygon& _curr) const
{
    for(unsigned int v_i = 0; v_i < 3; ++v_i)
        _segments.push({_prev[v_i], _curr[v_i]});

    if(m_segments_division_precision < 1)
        return;

    for(unsigned int v_i = 0; v_i < 3; ++v_i)
    {
        const glm::vec3& p_start = _prev[v_i];
        const glm::vec3& p_end = _prev[v_i + 1];
        const glm::vec3 p_piece = (p_end - p_start) * m_segment_piece_multiplier;

        const glm::vec3& c_start = _curr[v_i];
        const glm::vec3& c_end = _curr[v_i + 1];
        const glm::vec3 c_piece = (c_end - c_start) * m_segment_piece_multiplier;

        for(unsigned int i = 0; i < m_segments_division_precision; ++i)
        {
            glm::vec3 p_point = p_start + ( p_piece * (float)i );
            glm::vec3 c_point = c_start + ( c_piece * (float)i );

            _segments.push({p_point, c_point});
        }
    }
}

Dynamic_Narrow_CD__Mobile_Vs_Static::Ratio_Pair Dynamic_Narrow_CD__Mobile_Vs_Static::M_find_possible_collision_timeframe_for_segment(const Segment& _segment, const LDS::Vector<const Polygon*>& _polygons) const
{
    Ratio_Pair result;

    for(unsigned int i = 0; i < _polygons.size(); ++i)
    {
        const Polygon& polygon = *_polygons[i];

        if(LEti::Math::dot_product(_segment.direction(), polygon.calculate_normal()) > 0.0f)
            continue;

        Polygon_VS_Ray_Intersection_Data id = segment_intersects_polygon(_segment.start, _segment.end, polygon);
        if(!id.intersection)
            continue;

        result.consider_value(id.direction_ratio);
    }

    return result;
}

Dynamic_Narrow_CD__Mobile_Vs_Static::Ratio_Pair Dynamic_Narrow_CD__Mobile_Vs_Static::M_find_possible_collision_timeframe(const Physics_Module__Mesh& _mobile, const Physics_Module__Mesh& _static) const
{
    Ratio_Pair result;

    LDS::Vector<const Polygon*> static_poligons = M_find_possibly_colliding_polygons(_mobile.border(), _static);

    if(static_poligons.size() == 0)
        return result;

    const Polygon_Holder_Base& polygons_prev_frame = *_mobile.get_physical_model_prev_state()->get_polygons();
    const Polygon_Holder_Base& polygons_curr_frame = *_mobile.get_physical_model()->get_polygons();

    L_ASSERT(polygons_prev_frame.amount() == polygons_curr_frame.amount());

    unsigned int polygons_amount = polygons_prev_frame.amount();
    unsigned int segments_per_edge = 2 + m_segments_division_precision;
    unsigned int segments_per_polygon = segments_per_edge * 3;
    unsigned int segments_amount = polygons_amount * segments_per_polygon;

    LDS::Vector<Segment> segments(segments_amount);

    for(unsigned int p_i = 0; p_i < polygons_amount; ++p_i)
    {
        const Polygon& p_prev = *polygons_prev_frame.get_polygon(p_i);
        const Polygon& p_curr = *polygons_curr_frame.get_polygon(p_i);

        M_construct_polygon_segments(segments, p_prev, p_curr);
    }

    for(unsigned int i = 0; i < segments.size(); ++i)
    {
        Ratio_Pair ratios = M_find_possible_collision_timeframe_for_segment(segments[i], static_poligons);

        result.consider_value(ratios.min);
        result.consider_value(ratios.max);
    }

    if(!result.valid())
        result = {0.0f, 1.0f};

    return result;
}

Intersection_Data Dynamic_Narrow_CD__Mobile_Vs_Static::M_get_precise_time_ratio_of_collision(const Physics_Module__Mesh& _mobile, const Physics_Module__Mesh& _static, float _min_ratio, float _max_ratio) const
{
    L_ASSERT(!(_min_ratio < 0.0f || _max_ratio < 0.0f || _min_ratio > 1.0f || _max_ratio > 1.0f));

    float diff = _max_ratio - _min_ratio;
    float step_diff = diff / (float)m_interpolation_precision;

    Intersection_Data id;

    Physical_Model_Imprint first_impr = *_mobile.get_physical_model_prev_state();
    const Physical_Model& static_pm = *_static.get_physical_model();

    float curr_time_point = _min_ratio;
    while(curr_time_point <= _max_ratio)
    {
        LEti::Transformation_Data first_transform = LEti::Transformation_Data::get_transformation_data_for_ratio(*_mobile.transformation_data_prev_state(), *_mobile.transformation_data(), curr_time_point);

        first_impr.update_with_single_matrix(first_transform.matrix());

        id = m_intersection_detector->collision__model_vs_model(first_impr.get_polygons(),
                                                                first_impr.border(),
                                                                first_impr.polygons_borders(),
                                                                static_pm.get_polygons(),
                                                                static_pm.border(),
                                                                static_pm.polygons_borders());
        if(id)
            break;

        curr_time_point += step_diff;
    }

    if(!id && LEti::Math::floats_are_equal(_max_ratio, 1.0f))
        id = m_intersection_detector->collision__model_vs_model(_mobile.get_physical_model()->get_polygons(),
                                                                _mobile.get_physical_model()->border(),
                                                                _mobile.get_physical_model()->polygons_borders(),
                                                                _static.get_physical_model()->get_polygons(),
                                                                _static.get_physical_model()->border(),
                                                                _static.get_physical_model()->polygons_borders());

    if(id)
    {
        id.time_of_intersection_ratio = curr_time_point;
        if(id.time_of_intersection_ratio < 0.0f)
            id.time_of_intersection_ratio = 0.0f;
        if(id.time_of_intersection_ratio > 1.0f)
            id.time_of_intersection_ratio = 1.0f;
    }

    id.time_of_intersection_ratio_step = step_diff;

    return id;
}



Intersection_Data Dynamic_Narrow_CD__Mobile_Vs_Static::objects_collide(const Physics_Module__Mesh& _mobile, const Physics_Module__Mesh& _static) const
{
    if(m_interpolation_precision < 2)
    {
        Intersection_Data result = m_intersection_detector->collision__model_vs_model(_mobile.get_physical_model()->get_polygons(),
                                                                                      _mobile.get_physical_model()->border(),
                                                                                      _mobile.get_physical_model()->polygons_borders(),
                                                                                      _static.get_physical_model()->get_polygons(),
                                                                                      _static.get_physical_model()->border(),
                                                                                      _static.get_physical_model()->polygons_borders());

        if(!result)
            return Intersection_Data();

        result.time_of_intersection_ratio = 1.0f;
        return result;
    }

    Ratio_Pair possible_intersection_ratio = M_find_possible_collision_timeframe(_mobile, _static);

    if(possible_intersection_ratio.min > 1.0f)
        possible_intersection_ratio.min = 0.0f;
    else
        possible_intersection_ratio.min -= 0.02f;

    if(possible_intersection_ratio.max < 0.0f)
        possible_intersection_ratio.max = 1.0f;
    else
        possible_intersection_ratio.max += 0.02f;

    if(possible_intersection_ratio.min < 0.0f)
        possible_intersection_ratio.min = 0.0f;
    if(possible_intersection_ratio.max > 1.0f)
        possible_intersection_ratio.max = 1.0f;

    return M_get_precise_time_ratio_of_collision(_mobile, _static, possible_intersection_ratio.min, possible_intersection_ratio.max);
}



void Dynamic_Narrow_CD__Mobile_Vs_Static::update(const Broad_Phase_Interface::Colliding_Pair_List &_possible_collisions)
{
    m_collisions.clear();

    for(Broad_Phase_Interface::Colliding_Pair_List::Const_Iterator itm = _possible_collisions.begin(); !itm.end_reached(); ++itm)
    {
        L_ASSERT(LV::cast_variable<Physics_Module__Mesh>(itm->first));
        L_ASSERT(LV::cast_variable<Physics_Module__Mesh>(itm->second));

        Physics_Module__Mesh* first = (Physics_Module__Mesh*)itm->first;
        Physics_Module__Mesh* second = (Physics_Module__Mesh*)itm->second;

        L_ASSERT(first->is_static() ^ second->is_static());

        m_thread_pool.add_task([this, first, second]()
        {
            Intersection_Data id;

            if(second->is_static())
                id = objects_collide(*first, *second);
            else
                id = objects_collide(*second, *first);

            if(!id)
                return;

            id.first = first;
            id.second = second;

            m_save_intersection_mutex.lock();
            m_collisions.push_back(id);
            m_save_intersection_mutex.unlock();
        });
    }

    m_thread_pool.wait_for_completion();
}
