cmake_minimum_required(VERSION 3.5)

project(L_Physics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FILE(GLOB_RECURSE LibFiles "include/*.h")
add_custom_target(L_Physics_include SOURCES ${LibFiles})

file(GLOB_RECURSE Sources "source/*.cpp")

add_library(L_Physics STATIC ${Sources})

target_include_directories(L_Physics PUBLIC
                            "${PROJECT_SOURCE_DIR}"
                            "${PROJECT_SOURCE_DIR}/include"
                            "${PROJECT_SOURCE_DIR}/../L_Utility/include"
                            "${PROJECT_SOURCE_DIR}/../L_Variable/include"
                            "${PROJECT_SOURCE_DIR}/../LEti_Engine/include"
                            "${PROJECT_SOURCE_DIR}/../LEti_Engine/OpenGL/GLM"
)

