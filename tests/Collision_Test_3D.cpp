#include <Testing/Testing.h>

#include <Object.h>

#include <Modules/Physics_Module__Mesh.h>
#include <Collision_Detection/Narrow_Phase/Dynamic_Narrow_CD.h>
#include <Collision_Detection/Narrow_Phase/Dynamic_Narrow_CD__Mobile_Vs_Static.h>
#include <Collision_Detection/Primitives/SAT_Models_Intersection_3D.h>
#include <Collision_Resolution/Collision_Resolution__Physics_Module__Mesh.h>

INIT_TEST(Simple_Collision_Test_3D)
{
    glm::vec3 point_0 = {1, 0, 0};
    glm::vec3 point_1 = {0, 0, 1};
    glm::vec3 point_2 = {0, 1, -1};
    glm::vec3 point_3 = {0, -1, -1};

    float geometry_data[36] =
        {
            point_0.x, point_0.y, point_0.z,
            point_1.x, point_1.y, point_1.z,
            point_2.x, point_2.y, point_2.z,

            point_0.x, point_0.y, point_0.z,
            point_2.x, point_2.y, point_2.z,
            point_3.x, point_3.y, point_3.z,

            point_0.x, point_0.y, point_0.z,
            point_3.x, point_3.y, point_3.z,
            point_1.x, point_1.y, point_1.z,

            point_1.x, point_1.y, point_1.z,
            point_3.x, point_3.y, point_3.z,
            point_2.x, point_2.y, point_2.z
        };

    LPhys::Physics_Module_Stub__Mesh* mesh_pm = new LPhys::Physics_Module_Stub__Mesh;
    mesh_pm->coords.use_raw_data(geometry_data, 36);

    LEti::Object_Stub stub;
    stub.module_stubs.push_back({"Module__pm", mesh_pm});

    LEti::Object* object_0 = LEti::Object_Stub::construct_from(&stub);
    LEti::Object* object_1 = LEti::Object_Stub::construct_from(&stub);

    object_0->current_state().set_position({0.0f, 0.0f, 0.0f});
    object_1->current_state().set_position({0.99f, 0.0f, 0.0f});

    object_0->update(0.0f);
    object_0->update_previous_state();
    object_1->update(0.0f);
    object_1->update_previous_state();

    LPhys::Physical_Model* pm_0 = object_0->get_module_of_type<LPhys::Physics_Module__Mesh>()->get_physical_model();
    LPhys::Physical_Model* pm_1 = object_1->get_module_of_type<LPhys::Physics_Module__Mesh>()->get_physical_model();

    LPhys::SAT_Models_Intersection_3D collision_detection;
    LPhys::Intersection_Data intersection_data = collision_detection.collision__model_vs_model(pm_0->get_polygons(), pm_0->border(), pm_0->polygons_borders(),
                                                                                               pm_1->get_polygons(), pm_1->border(), pm_1->polygons_borders());

    EXPECT_EQUAL(intersection_data.intersection, true);

    delete object_0;
    delete object_1;
}

INIT_TEST(Collision_Movement_Test_3D)
{
    glm::vec3 point_0 = {1, 0, 0};
    glm::vec3 point_1 = {0, 0, 1};
    glm::vec3 point_2 = {0, 1, -1};
    glm::vec3 point_3 = {0, -1, -1};

    float geometry_data[36] =
        {
            point_0.x, point_0.y, point_0.z,
            point_2.x, point_2.y, point_2.z,
            point_1.x, point_1.y, point_1.z,

            point_0.x, point_0.y, point_0.z,
            point_3.x, point_3.y, point_3.z,
            point_2.x, point_2.y, point_2.z,

            point_0.x, point_0.y, point_0.z,
            point_1.x, point_1.y, point_1.z,
            point_3.x, point_3.y, point_3.z,

            point_1.x, point_1.y, point_1.z,
            point_2.x, point_2.y, point_2.z,
            point_3.x, point_3.y, point_3.z
        };

    LPhys::Physics_Module_Stub__Mesh* mesh_pm = new LPhys::Physics_Module_Stub__Mesh;
    mesh_pm->coords.use_raw_data(geometry_data, 36);

    LEti::Object_Stub stub;
    stub.module_stubs.push_back({"Module__pm", mesh_pm});

    LEti::Object* object_0 = LEti::Object_Stub::construct_from(&stub);
    LEti::Object* object_1 = LEti::Object_Stub::construct_from(&stub);

    object_0->current_state().set_position({0.0f, 0.0f, 0.0f});
    object_0->update(0.0f);
    object_0->update_previous_state();
    object_0->current_state().set_position({1.5f, 0.0f, 0.0f});
    object_0->update(0.0f);

    object_1->current_state().set_position({2.0f, 0.0f, 0.0f});
    object_1->update(0.0f);
    object_1->update_previous_state();

    LPhys::Physics_Module__Mesh* pm_0 = object_0->get_module_of_type<LPhys::Physics_Module__Mesh>();
    LPhys::Physics_Module__Mesh* pm_1 = object_1->get_module_of_type<LPhys::Physics_Module__Mesh>();

    LPhys::Dynamic_Narrow_CD dynamic_cd;
    dynamic_cd.set_intersection_detector(new LPhys::SAT_Models_Intersection_3D);

    LPhys::Intersection_Data intersection_data = dynamic_cd.objects_collide(*pm_0, *pm_1);

    EXPECT_EQUAL(intersection_data.intersection, true);

    intersection_data.first = pm_0;
    intersection_data.second = pm_1;

    LPhys::Collision_Resolution__Physics_Module__Mesh collision_resolution;
    collision_resolution.resolve(intersection_data);

    pm_0->apply_data_after_collisions();
    pm_1->apply_data_after_collisions();

    object_0->update_previous_state();
    object_0->update(0.0f);
    object_1->update_previous_state();
    object_1->update(0.0f);

    pm_0->update_physical_model();
    pm_1->update_physical_model();

    intersection_data = dynamic_cd.objects_collide(*pm_0, *pm_1);

    EXPECT_EQUAL(intersection_data.intersection, false);

    delete object_0;
    delete object_1;
}

INIT_TEST(Collision_Extream_Movement_Test_3D)
{
    glm::vec3 point_0 = {1, 0, 0};
    glm::vec3 point_1 = {0, 0, 1};
    glm::vec3 point_2 = {0, 1, -1};
    glm::vec3 point_3 = {0, -1, -1};

    float geometry_data[36] =
        {
            point_0.x, point_0.y, point_0.z,
            point_2.x, point_2.y, point_2.z,
            point_1.x, point_1.y, point_1.z,

            point_0.x, point_0.y, point_0.z,
            point_3.x, point_3.y, point_3.z,
            point_2.x, point_2.y, point_2.z,

            point_0.x, point_0.y, point_0.z,
            point_1.x, point_1.y, point_1.z,
            point_3.x, point_3.y, point_3.z,

            point_1.x, point_1.y, point_1.z,
            point_2.x, point_2.y, point_2.z,
            point_3.x, point_3.y, point_3.z
        };

    LPhys::Physics_Module_Stub__Mesh* mesh_pm = new LPhys::Physics_Module_Stub__Mesh;
    mesh_pm->coords.use_raw_data(geometry_data, 36);

    LEti::Object_Stub stub;
    stub.module_stubs.push_back({"Module__pm", mesh_pm});

    LEti::Object* object_0 = LEti::Object_Stub::construct_from(&stub);
    LEti::Object* object_1 = LEti::Object_Stub::construct_from(&stub);

    object_0->current_state().set_position({0.0f, 0.0f, 0.0f});
    object_0->update(0.0f);
    object_0->update_previous_state();
    object_0->current_state().set_position({10.0f, 0.0f, 0.0f});
    object_0->update(0.0f);

    object_1->current_state().set_position({5.0f, 0.0f, 0.0f});
    object_1->update(0.0f);
    object_1->update_previous_state();

    LPhys::Physics_Module__Mesh* pm_0 = object_0->get_module_of_type<LPhys::Physics_Module__Mesh>();
    LPhys::Physics_Module__Mesh* pm_1 = object_1->get_module_of_type<LPhys::Physics_Module__Mesh>();

    LPhys::Dynamic_Narrow_CD dynamic_cd;
    dynamic_cd.set_intersection_detector(new LPhys::SAT_Models_Intersection_3D);

    LPhys::Intersection_Data intersection_data = dynamic_cd.objects_collide(*pm_0, *pm_1);

    EXPECT_EQUAL(intersection_data.intersection, true);

    intersection_data.first = pm_0;
    intersection_data.second = pm_1;

    LPhys::Collision_Resolution__Physics_Module__Mesh collision_resolution;
    collision_resolution.resolve(intersection_data);

    pm_0->apply_data_after_collisions();
    pm_1->apply_data_after_collisions();

    object_0->update_previous_state();
    object_0->update(0.0f);
    object_1->update_previous_state();
    object_1->update(0.0f);

    pm_0->update_physical_model();
    pm_1->update_physical_model();

    intersection_data = dynamic_cd.objects_collide(*pm_0, *pm_1);

    EXPECT_EQUAL(intersection_data.intersection, false);

    delete object_0;
    delete object_1;
}

INIT_TEST(Collision_Movement_Dynamic_Vs_Static_Test_3D)
{
    glm::vec3 point_0 = {1, 0, 0};
    glm::vec3 point_1 = {0, 0, 1};
    glm::vec3 point_2 = {0, 1, -1};
    glm::vec3 point_3 = {0, -1, -1};

    float geometry_data[36] =
        {
            point_0.x, point_0.y, point_0.z,
            point_2.x, point_2.y, point_2.z,
            point_1.x, point_1.y, point_1.z,

            point_0.x, point_0.y, point_0.z,
            point_3.x, point_3.y, point_3.z,
            point_2.x, point_2.y, point_2.z,

            point_0.x, point_0.y, point_0.z,
            point_1.x, point_1.y, point_1.z,
            point_3.x, point_3.y, point_3.z,

            point_1.x, point_1.y, point_1.z,
            point_2.x, point_2.y, point_2.z,
            point_3.x, point_3.y, point_3.z
        };

    LPhys::Physics_Module_Stub__Mesh* mesh_pm = new LPhys::Physics_Module_Stub__Mesh;
    mesh_pm->coords.use_raw_data(geometry_data, 36);

    LEti::Object_Stub stub;
    stub.module_stubs.push_back({"Module__pm", mesh_pm});

    LEti::Object* object_0 = LEti::Object_Stub::construct_from(&stub);
    LEti::Object* object_1 = LEti::Object_Stub::construct_from(&stub);

    object_0->current_state().set_position({0.0f, 0.0f, 0.0f});
    object_0->update(0.0f);
    object_0->update_previous_state();
    object_0->current_state().set_position({1.5f, 0.0f, 0.0f});
    object_0->update(0.0f);

    object_1->current_state().set_position({2.0f, 0.0f, 0.0f});
    object_1->update(0.0f);
    object_1->update_previous_state();

    LPhys::Physics_Module__Mesh* pm_0 = object_0->get_module_of_type<LPhys::Physics_Module__Mesh>();
    LPhys::Physics_Module__Mesh* pm_1 = object_1->get_module_of_type<LPhys::Physics_Module__Mesh>();
    pm_1->set_static(true);

    LPhys::Dynamic_Narrow_CD__Mobile_Vs_Static dynamic_cd;
    dynamic_cd.set_intersection_detector(new LPhys::SAT_Models_Intersection_3D);

    LPhys::Intersection_Data intersection_data = dynamic_cd.objects_collide(*pm_0, *pm_1);

    EXPECT_EQUAL(intersection_data.intersection, true);

    intersection_data.first = pm_0;
    intersection_data.second = pm_1;

    LPhys::Collision_Resolution__Physics_Module__Mesh collision_resolution;
    collision_resolution.resolve(intersection_data);

    pm_0->apply_data_after_collisions();
    pm_1->apply_data_after_collisions();

    object_0->update_previous_state();
    object_0->update(0.0f);
    object_1->update_previous_state();
    object_1->update(0.0f);

    pm_0->update_physical_model();
    pm_1->update_physical_model();

    intersection_data = dynamic_cd.objects_collide(*pm_0, *pm_1);

    EXPECT_EQUAL(intersection_data.intersection, false);

    delete object_0;
    delete object_1;
}

INIT_TEST(Collision_Extream_Movement_Dynamic_Vs_Static_Test_3D)
{
    glm::vec3 point_0 = {1, 0, 0};
    glm::vec3 point_1 = {0, 0, 1};
    glm::vec3 point_2 = {0, 1, -1};
    glm::vec3 point_3 = {0, -1, -1};

    float geometry_data[36] =
        {
            point_0.x, point_0.y, point_0.z,
            point_2.x, point_2.y, point_2.z,
            point_1.x, point_1.y, point_1.z,

            point_0.x, point_0.y, point_0.z,
            point_3.x, point_3.y, point_3.z,
            point_2.x, point_2.y, point_2.z,

            point_0.x, point_0.y, point_0.z,
            point_1.x, point_1.y, point_1.z,
            point_3.x, point_3.y, point_3.z,

            point_1.x, point_1.y, point_1.z,
            point_2.x, point_2.y, point_2.z,
            point_3.x, point_3.y, point_3.z
        };

    LPhys::Physics_Module_Stub__Mesh* mesh_pm = new LPhys::Physics_Module_Stub__Mesh;
    mesh_pm->coords.use_raw_data(geometry_data, 36);

    LEti::Object_Stub stub;
    stub.module_stubs.push_back({"Module__pm", mesh_pm});

    LEti::Object* object_0 = LEti::Object_Stub::construct_from(&stub);
    LEti::Object* object_1 = LEti::Object_Stub::construct_from(&stub);

    object_0->current_state().set_position({0.0f, 0.0f, 0.0f});
    object_0->update(0.0f);
    object_0->update_previous_state();
    object_0->current_state().set_position({10.0f, 0.0f, 0.0f});
    object_0->update(0.0f);

    object_1->current_state().set_position({5.0f, 0.0f, 0.0f});
    object_1->update(0.0f);
    object_1->update_previous_state();

    LPhys::Physics_Module__Mesh* pm_0 = object_0->get_module_of_type<LPhys::Physics_Module__Mesh>();
    LPhys::Physics_Module__Mesh* pm_1 = object_1->get_module_of_type<LPhys::Physics_Module__Mesh>();
    pm_1->set_static(true);

    LPhys::Dynamic_Narrow_CD__Mobile_Vs_Static dynamic_cd;
    dynamic_cd.set_intersection_detector(new LPhys::SAT_Models_Intersection_3D);

    LPhys::Intersection_Data intersection_data = dynamic_cd.objects_collide(*pm_0, *pm_1);

    EXPECT_EQUAL(intersection_data.intersection, true);

    intersection_data.first = pm_0;
    intersection_data.second = pm_1;

    LPhys::Collision_Resolution__Physics_Module__Mesh collision_resolution;
    collision_resolution.resolve(intersection_data);

    pm_0->apply_data_after_collisions();
    pm_1->apply_data_after_collisions();

    object_0->update_previous_state();
    object_0->update(0.0f);
    object_1->update_previous_state();
    object_1->update(0.0f);

    pm_0->update_physical_model();
    pm_1->update_physical_model();

    intersection_data = dynamic_cd.objects_collide(*pm_0, *pm_1);

    EXPECT_EQUAL(intersection_data.intersection, false);

    delete object_0;
    delete object_1;
}
