#include <Testing/Testing.h>

#include <Object.h>

#include <Modules/Physics_Module__Mesh.h>
#include <Collision_Detection/Narrow_Phase/Dynamic_Narrow_CD.h>
#include <Collision_Detection/Narrow_Phase/Dynamic_Narrow_CD__Mobile_Vs_Static.h>
#include <Collision_Detection/Primitives/SAT_Models_Intersection_3D.h>
#include <Collision_Resolution/Collision_Resolution__Physics_Module__Mesh.h>

INIT_TEST(Collision_Test_3D_Edging)
{
    glm::vec3 point_0_dynamic = {1, 1, 1};
    glm::vec3 point_1_dynamic = {0, 1, -1};
    glm::vec3 point_2_dynamic = {-1, 1, 1};
    glm::vec3 point_3_dynamic = {0, 0, 0};

    float geometry_data_dynamic[36] =
        {
            point_0_dynamic.x, point_0_dynamic.y, point_0_dynamic.z,
            point_1_dynamic.x, point_1_dynamic.y, point_1_dynamic.z,
            point_2_dynamic.x, point_2_dynamic.y, point_2_dynamic.z,

            point_0_dynamic.x, point_0_dynamic.y, point_0_dynamic.z,
            point_2_dynamic.x, point_2_dynamic.y, point_2_dynamic.z,
            point_3_dynamic.x, point_3_dynamic.y, point_3_dynamic.z,

            point_0_dynamic.x, point_0_dynamic.y, point_0_dynamic.z,
            point_3_dynamic.x, point_3_dynamic.y, point_3_dynamic.z,
            point_1_dynamic.x, point_1_dynamic.y, point_1_dynamic.z,

            point_1_dynamic.x, point_1_dynamic.y, point_1_dynamic.z,
            point_3_dynamic.x, point_3_dynamic.y, point_3_dynamic.z,
            point_2_dynamic.x, point_2_dynamic.y, point_2_dynamic.z
        };

    LPhys::Physics_Module_Stub__Mesh* mesh_pm_dynamic = new LPhys::Physics_Module_Stub__Mesh;
    mesh_pm_dynamic->coords.use_raw_data(geometry_data_dynamic, 36);

    LEti::Object_Stub stub_dynamic;
    stub_dynamic.module_stubs.push_back({"Module__pm", mesh_pm_dynamic});

    LEti::Object* object_dynamic = LEti::Object_Stub::construct_from(&stub_dynamic);

    glm::vec3 point_0_static = {10, 0, 10};
    glm::vec3 point_1_static = {10, 0, -10};
    glm::vec3 point_2_static = {-10, 0, -10};
    glm::vec3 point_3_static = {-10, 0, 10};

    float geometry_data_static[18] =
        {
            point_0_static.x, point_0_static.y, point_0_static.z,
            point_1_static.x, point_1_static.y, point_1_static.z,
            point_2_static.x, point_2_static.y, point_2_static.z,

            point_0_static.x, point_0_static.y, point_0_static.z,
            point_2_static.x, point_2_static.y, point_2_static.z,
            point_3_static.x, point_3_static.y, point_3_static.z,
        };

    LPhys::Physics_Module_Stub__Mesh* mesh_pm_static = new LPhys::Physics_Module_Stub__Mesh;
    mesh_pm_static->coords.use_raw_data(geometry_data_static, 36);

    LEti::Object_Stub stub_static;
    stub_static.module_stubs.push_back({"Module__pm", mesh_pm_static});

    LEti::Object* object_static_0 = LEti::Object_Stub::construct_from(&stub_static);
    LEti::Object* object_static_1 = LEti::Object_Stub::construct_from(&stub_static);


    object_dynamic->current_state().set_position({0.0f, 0.2f, -10.0f});
    object_static_0->current_state().set_position({0.0f, 0.0f, -10.0f});
    object_static_1->current_state().set_position({0.0f, 0.0f, 10.0f});

    object_dynamic->update(0.0f);
    object_dynamic->update_previous_state();
    object_static_0->update(0.0f);
    object_static_0->update_previous_state();
    object_static_1->update(0.0f);
    object_static_1->update_previous_state();

    LPhys::Physics_Module__Mesh* pm_dynamic = object_dynamic->get_module_of_type<LPhys::Physics_Module__Mesh>();
    LPhys::Physics_Module__Mesh* pm_static_0 = object_static_0->get_module_of_type<LPhys::Physics_Module__Mesh>();
    LPhys::Physics_Module__Mesh* pm_static_1 = object_static_1->get_module_of_type<LPhys::Physics_Module__Mesh>();
    pm_static_0->set_static(true);
    pm_static_1->set_static(true);

    LPhys::Dynamic_Narrow_CD__Mobile_Vs_Static dynamic_cd;
    dynamic_cd.set_intersection_detector(new LPhys::SAT_Models_Intersection_3D);

    LPhys::Collision_Resolution__Physics_Module__Mesh collision_resolution;

    {
        LPhys::Intersection_Data intersection_data = dynamic_cd.objects_collide(*pm_dynamic, *pm_static_0);
        EXPECT_EQUAL(intersection_data.intersection, false);
        intersection_data = dynamic_cd.objects_collide(*pm_dynamic, *pm_static_1);
        EXPECT_EQUAL(intersection_data.intersection, false);
    }

    for(unsigned int i = 0; i < 400; ++i)
    {
        object_dynamic->update_previous_state();
        object_static_0->update_previous_state();
        object_static_1->update_previous_state();

        object_dynamic->current_state().move({0.0f, -0.05f, 0.05f});

        object_dynamic->update(0.0f);
        object_static_0->update(0.0f);
        object_static_1->update(0.0f);

        LPhys::Intersection_Data intersection_data_0 = dynamic_cd.objects_collide(*pm_dynamic, *pm_static_0);
        LPhys::Intersection_Data intersection_data_1 = dynamic_cd.objects_collide(*pm_dynamic, *pm_static_1);

        if(intersection_data_0)
        {
            intersection_data_0.first = pm_dynamic;
            intersection_data_0.second = pm_static_0;

            collision_resolution.resolve(intersection_data_0);
        }
        if(intersection_data_1)
        {
            intersection_data_1.first = pm_dynamic;
            intersection_data_1.second = pm_static_1;

            collision_resolution.resolve(intersection_data_1);
        }

        pm_dynamic->apply_data_after_collisions();
        pm_static_0->apply_data_after_collisions();
        pm_static_1->apply_data_after_collisions();

        EXPECT_MORE_OR_EQUAL(object_dynamic->current_state().position().y, 0.0f);
    }

    EXPECT_MORE(object_dynamic->current_state().position().z, 1.0f);

    delete object_dynamic;
    delete object_static_0;
    delete object_static_1;
}

INIT_TEST(Collision_Test_3D_Edging_Flat)
{
    glm::vec3 point_0_dynamic = {1, 0, 0};
    glm::vec3 point_1_dynamic = {0, 1, 0};
    glm::vec3 point_2_dynamic = {0, 0, 1};
    glm::vec3 point_3_dynamic = {0, 0, -1};

    float geometry_data_dynamic[36] =
        {
            point_0_dynamic.x, point_0_dynamic.y, point_0_dynamic.z,
            point_1_dynamic.x, point_1_dynamic.y, point_1_dynamic.z,
            point_2_dynamic.x, point_2_dynamic.y, point_2_dynamic.z,

            point_0_dynamic.x, point_0_dynamic.y, point_0_dynamic.z,
            point_2_dynamic.x, point_2_dynamic.y, point_2_dynamic.z,
            point_3_dynamic.x, point_3_dynamic.y, point_3_dynamic.z,

            point_0_dynamic.x, point_0_dynamic.y, point_0_dynamic.z,
            point_3_dynamic.x, point_3_dynamic.y, point_3_dynamic.z,
            point_1_dynamic.x, point_1_dynamic.y, point_1_dynamic.z,

            point_1_dynamic.x, point_1_dynamic.y, point_1_dynamic.z,
            point_3_dynamic.x, point_3_dynamic.y, point_3_dynamic.z,
            point_2_dynamic.x, point_2_dynamic.y, point_2_dynamic.z
        };

    LPhys::Physics_Module_Stub__Mesh* mesh_pm_dynamic = new LPhys::Physics_Module_Stub__Mesh;
    mesh_pm_dynamic->coords.use_raw_data(geometry_data_dynamic, 36);

    LEti::Object_Stub stub_dynamic;
    stub_dynamic.module_stubs.push_back({"Module__pm", mesh_pm_dynamic});

    LEti::Object* object_dynamic = LEti::Object_Stub::construct_from(&stub_dynamic);

    glm::vec3 point_0_static = {10, 0, 10};
    glm::vec3 point_1_static = {10, 0, -10};
    glm::vec3 point_2_static = {-10, 0, -10};
    glm::vec3 point_3_static = {-10, 0, 10};

    float geometry_data_static[18] =
        {
            point_0_static.x, point_0_static.y, point_0_static.z,
            point_1_static.x, point_1_static.y, point_1_static.z,
            point_2_static.x, point_2_static.y, point_2_static.z,

            point_0_static.x, point_0_static.y, point_0_static.z,
            point_2_static.x, point_2_static.y, point_2_static.z,
            point_3_static.x, point_3_static.y, point_3_static.z,
        };

    LPhys::Physics_Module_Stub__Mesh* mesh_pm_static = new LPhys::Physics_Module_Stub__Mesh;
    mesh_pm_static->coords.use_raw_data(geometry_data_static, 18);

    LEti::Object_Stub stub_static;
    stub_static.module_stubs.push_back({"Module__pm", mesh_pm_static});

    LEti::Object* object_static_0 = LEti::Object_Stub::construct_from(&stub_static);
    LEti::Object* object_static_1 = LEti::Object_Stub::construct_from(&stub_static);


    object_dynamic->current_state().set_position({0.0f, 0.2f, -10.0f});
    object_static_0->current_state().set_position({0.0f, 0.0f, -10.0f});
    object_static_1->current_state().set_position({0.0f, 0.0f, 10.0f});

    object_dynamic->update(0.0f);
    object_dynamic->update_previous_state();
    object_static_0->update(0.0f);
    object_static_0->update_previous_state();
    object_static_1->update(0.0f);
    object_static_1->update_previous_state();

    LPhys::Physics_Module__Mesh* pm_dynamic = object_dynamic->get_module_of_type<LPhys::Physics_Module__Mesh>();
    LPhys::Physics_Module__Mesh* pm_static_0 = object_static_0->get_module_of_type<LPhys::Physics_Module__Mesh>();
    LPhys::Physics_Module__Mesh* pm_static_1 = object_static_1->get_module_of_type<LPhys::Physics_Module__Mesh>();
    pm_static_0->set_static(true);
    pm_static_1->set_static(true);

    LPhys::Dynamic_Narrow_CD__Mobile_Vs_Static dynamic_cd;
    dynamic_cd.set_intersection_detector(new LPhys::SAT_Models_Intersection_3D);

    LPhys::Collision_Resolution__Physics_Module__Mesh collision_resolution;

    {
        LPhys::Intersection_Data intersection_data = dynamic_cd.objects_collide(*pm_dynamic, *pm_static_0);
        EXPECT_EQUAL(intersection_data.intersection, false);
        intersection_data = dynamic_cd.objects_collide(*pm_dynamic, *pm_static_1);
        EXPECT_EQUAL(intersection_data.intersection, false);
    }

    for(unsigned int i = 0; i < 400; ++i)
    {
        object_dynamic->update_previous_state();
        object_static_0->update_previous_state();
        object_static_1->update_previous_state();

        object_dynamic->current_state().move({0.0f, -0.05f, 0.05f});

        object_dynamic->update(0.0f);
        object_static_0->update(0.0f);
        object_static_1->update(0.0f);

        LPhys::Intersection_Data intersection_data_0 = dynamic_cd.objects_collide(*pm_dynamic, *pm_static_0);
        LPhys::Intersection_Data intersection_data_1 = dynamic_cd.objects_collide(*pm_dynamic, *pm_static_1);

        if(intersection_data_0)
        {
            intersection_data_0.first = pm_dynamic;
            intersection_data_0.second = pm_static_0;

            collision_resolution.resolve(intersection_data_0);
        }
        if(intersection_data_1)
        {
            intersection_data_1.first = pm_dynamic;
            intersection_data_1.second = pm_static_1;

            collision_resolution.resolve(intersection_data_1);
        }

        pm_dynamic->apply_data_after_collisions();
        pm_static_0->apply_data_after_collisions();
        pm_static_1->apply_data_after_collisions();

        EXPECT_MORE_OR_EQUAL(object_dynamic->current_state().position().y, 0.0f);
    }

    EXPECT_MORE(object_dynamic->current_state().position().z, 1.0f);

    delete object_dynamic;
    delete object_static_0;
    delete object_static_1;
}
