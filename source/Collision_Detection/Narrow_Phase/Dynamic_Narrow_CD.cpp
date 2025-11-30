#include <Collision_Detection/Narrow_Phase/Dynamic_Narrow_CD.h>

using namespace LPhys;


Dynamic_Narrow_CD::Dynamic_Narrow_CD()
{

}

Dynamic_Narrow_CD::~Dynamic_Narrow_CD()
{
    delete m_intersection_detector;
}



Dynamic_Narrow_CD::Ratio_Pair Dynamic_Narrow_CD::M_find_possible_collision_timeframe(const Physics_Module__Mesh& _first, const Physics_Module__Mesh& _second) const
{
    Ratio_Pair result;

    Physical_Model_Imprint ppm1_relative_prev = *_first.get_physical_model_prev_state();
    Physical_Model_Imprint ppm1_relative = ppm1_relative_prev;

    constexpr glm::mat4x4 fake_default_matrix{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    glm::vec3 pos_diff_vector_prev = _first.transformation_data_prev_state()->position() - _second.transformation_data_prev_state()->position();
    pos_diff_vector_prev = LEti::Transformation_Data::get_rotation_matrix_inversed_for_ratio(*_second.transformation_data_prev_state(), *_second.transformation_data(), 0.0f) * glm::vec4(pos_diff_vector_prev, 1.0f);

    glm::mat4x4 diff_pos_prev = fake_default_matrix;
    for(unsigned int i=0; i<3; ++i)
        diff_pos_prev[3][i] += pos_diff_vector_prev[i];

    glm::mat4x4 diff_rotation_prev = LEti::Transformation_Data::get_rotation_matrix_for_ratio(*_first.transformation_data_prev_state(), *_first.transformation_data(), 0.0f) / LEti::Transformation_Data::get_rotation_matrix_for_ratio(*_second.transformation_data_prev_state(), *_second.transformation_data(), 0.0f);
    glm::mat4x4 diff_scale_prev = LEti::Transformation_Data::get_scale_matrix_for_ratio(*_first.transformation_data_prev_state(), *_first.transformation_data(), 0.0f);


    glm::vec3 pos_diff_vector = _first.transformation_data()->position() - _second.transformation_data()->position();
    pos_diff_vector = LEti::Transformation_Data::get_rotation_matrix_inversed_for_ratio(*_second.transformation_data_prev_state(), *_second.transformation_data(), 1.0f) * glm::vec4(pos_diff_vector, 1.0f);

    glm::mat4x4 diff_pos = fake_default_matrix;
    for(unsigned int i=0; i<3; ++i)
        diff_pos[3][i] += pos_diff_vector[i];

    glm::mat4x4 diff_rotation = LEti::Transformation_Data::get_rotation_matrix_for_ratio(*_first.transformation_data_prev_state(), *_first.transformation_data(), 1.0f) / LEti::Transformation_Data::get_rotation_matrix_for_ratio(*_second.transformation_data_prev_state(), *_second.transformation_data(), 1.0f);

    glm::vec3 m2_scale_diff_vec = _second.transformation_data()->scale() - _second.transformation_data_prev_state()->scale();
    glm::mat4x4 m2_scale_diff_matrix = fake_default_matrix;
    for(unsigned int i=0; i<3; ++i)
        m2_scale_diff_matrix[i][i] += m2_scale_diff_vec[i];

    glm::mat4x4 diff_scale = LEti::Transformation_Data::get_scale_matrix_for_ratio(*_first.transformation_data_prev_state(), *_first.transformation_data(), 1.0f) * m2_scale_diff_matrix;

    ppm1_relative_prev.update(diff_pos_prev, diff_rotation_prev, diff_scale_prev);
    ppm1_relative.update(diff_pos, diff_rotation, diff_scale);

    Physical_Model_Imprint ppm2 = *_second.get_physical_model_prev_state();
    ppm2.update(fake_default_matrix, fake_default_matrix, LEti::Transformation_Data::get_scale_matrix_for_ratio(*_second.transformation_data_prev_state(), *_second.transformation_data(), 0.0f));

    const Border& relative_border_1_prev = ppm1_relative_prev.border();
    const Border& relative_border_1 = ppm1_relative.border();

    const Border& border_2 = ppm2.border();

    for(unsigned int i=0; i<3; ++i)
    {
        float stride_of_min = (relative_border_1.offset()[i]) - (relative_border_1_prev.offset()[i]);
        float stride_of_max = (relative_border_1.offset()[i] + relative_border_1.size()[i]) - (relative_border_1_prev.offset()[i] + relative_border_1_prev.size()[i]);

        if(!LEti::Math::floats_are_equal(stride_of_max, 0.0f))
            result.consider_value(( border_2.offset()[i] - (relative_border_1.offset()[i] + relative_border_1.size()[i]) ) / stride_of_max);
        if(!LEti::Math::floats_are_equal(stride_of_min, 0.0f))
            result.consider_value(( (border_2.offset()[i] + border_2.size()[i]) - relative_border_1_prev.offset()[i] ) / stride_of_min);
    }

    return result;
}

Intersection_Data Dynamic_Narrow_CD::M_get_precise_time_ratio_of_collision(const Physics_Module__Mesh& _first, const Physics_Module__Mesh& _second, float _min_ratio, float _max_ratio) const
{
    L_ASSERT(!(_min_ratio < 0.0f || _max_ratio < 0.0f || _min_ratio > 1.0f || _max_ratio > 1.0f));

    float diff = _max_ratio - _min_ratio;
    float step_diff = diff / (float)m_precision;

    Intersection_Data id;

    Physical_Model_Imprint first_impr = *_first.get_physical_model_prev_state();
    Physical_Model_Imprint second_impr = *_second.get_physical_model_prev_state();

    float curr_time_point = _min_ratio;
    while(curr_time_point <= _max_ratio)
    {
        LEti::Transformation_Data first_transform = LEti::Transformation_Data::get_transformation_data_for_ratio(*_first.transformation_data_prev_state(), *_first.transformation_data(), curr_time_point);
        LEti::Transformation_Data second_transform = LEti::Transformation_Data::get_transformation_data_for_ratio(*_second.transformation_data_prev_state(), *_second.transformation_data(), curr_time_point);

        first_impr.update_with_single_matrix(first_transform.matrix());
        second_impr.update_with_single_matrix(second_transform.matrix());

        id = m_intersection_detector->collision__model_vs_model(first_impr.get_polygons(), first_impr.border(), second_impr.get_polygons(), second_impr.border());
        if(id)
            break;

        curr_time_point += step_diff;
    }

    if(!id && LEti::Math::floats_are_equal(_max_ratio, 1.0f))
        id = m_intersection_detector->collision__model_vs_model(_first.get_physical_model()->get_polygons(), _first.get_physical_model()->border(), _second.get_physical_model()->get_polygons(), _second.get_physical_model()->border());

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



Intersection_Data Dynamic_Narrow_CD::objects_collide(const Physics_Module__Mesh& _first, const Physics_Module__Mesh& _second) const
{
    if(m_precision < 2)
    {
        Intersection_Data result = m_intersection_detector->collision__model_vs_model(_first.get_physical_model()->get_polygons(),
                                                                                      _first.get_physical_model()->border(),
                                                                                      _second.get_physical_model()->get_polygons(),
                                                                                      _second.get_physical_model()->border());

        if(!result)
            return Intersection_Data();

        result.time_of_intersection_ratio = 1.0f;
        return result;
    }

    Ratio_Pair possible_intersection_ratio = M_find_possible_collision_timeframe(_first, _second);
    Ratio_Pair possible_intersection_ratio_reverse = M_find_possible_collision_timeframe(_second, _first);
    if(possible_intersection_ratio.min > possible_intersection_ratio_reverse.min)
        possible_intersection_ratio.min = possible_intersection_ratio_reverse.min;
    if(possible_intersection_ratio.max < possible_intersection_ratio_reverse.max)
        possible_intersection_ratio.max = possible_intersection_ratio_reverse.max;

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

    return M_get_precise_time_ratio_of_collision(_first, _second, possible_intersection_ratio.min, possible_intersection_ratio.max);
}



void Dynamic_Narrow_CD::update(const Broad_Phase_Interface::Colliding_Pair_List &_possible_collisions)
{
    m_collisions.clear();

    for(Broad_Phase_Interface::Colliding_Pair_List::Const_Iterator itm = _possible_collisions.begin(); !itm.end_reached(); ++itm)
    {
        L_ASSERT(LV::cast_variable<Physics_Module__Mesh>(itm->first));
        L_ASSERT(LV::cast_variable<Physics_Module__Mesh>(itm->second));

        Physics_Module__Mesh* first = (Physics_Module__Mesh*)itm->first;
        Physics_Module__Mesh* second = (Physics_Module__Mesh*)itm->second;

        m_thread_pool.add_task([this, first, second]()
        {
            Intersection_Data id = objects_collide(*first, *second);
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
