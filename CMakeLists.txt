cmake_minimum_required(VERSION 3.5)

project(L_Physics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FILE(GLOB_RECURSE LibFiles "include/*.h")
add_custom_target(L_Physics_include SOURCES ${LibFiles})

add_library(L_Physics STATIC
    source/Modules/Physics_Module_2D.cpp
    source/Modules/Rigid_Body_2D.cpp
    source/Physical_Models/Physical_Model_2D.cpp
    source/Physical_Models/Polygon.cpp
    source/Physical_Models/Rigid_Body_Physical_Model_2D.cpp
    source/Physical_Models/Rigid_Body_Polygon.cpp
    source/Collision_Resolution/Collision_Resolver.cpp
    source/Collision_Resolution/Collision_Resolution__Rigid_Body_2D.cpp
    source/Collision_Detection/Intersection_Data.cpp
    source/Collision_Detection/Broad_Phase_Interface.cpp
    source/Collision_Detection/Collision_Detector_2D.cpp
    source/Collision_Detection/Default_Narrowest_CD.cpp
    source/Collision_Detection/Dynamic_Narrow_CD.cpp
    source/Collision_Detection/Narrow_Phase_Interface.cpp
    source/Collision_Detection/Narrowest_Phase_Interface.cpp
    source/Collision_Detection/SAT_Narrowest_CD.cpp
    source/Collision_Detection/Space_Hasher_2D.cpp
)

target_include_directories(L_Physics PUBLIC
                            "${PROJECT_SOURCE_DIR}"
                            "${PROJECT_SOURCE_DIR}/include"
                            "${PROJECT_SOURCE_DIR}/../L_Utility/include"
                            "${PROJECT_SOURCE_DIR}/../L_Variable/include"
                            "${PROJECT_SOURCE_DIR}/../LEti_Engine/include"
                            "${PROJECT_SOURCE_DIR}/../LEti_Engine/OpenGL/GLM"
)

