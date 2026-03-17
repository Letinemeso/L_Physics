file(GLOB_RECURSE Sources "./*.cpp")

if(MSVC)
    add_compile_options(/bigobj)
endif(MSVC)

add_executable(L_Physics_Tests ${Sources})

# add_subdirectory("${PROJECT_SOURCE_DIR}/../L_Utility"
#                  "${PROJECT_BINARY_DIR}/../L_Utility"
# )
# add_subdirectory("${PROJECT_SOURCE_DIR}/../L_Variable"
#                  "${PROJECT_BINARY_DIR}/../L_Variable"
# )
# add_subdirectory("${PROJECT_SOURCE_DIR}/../LEti_Engine"
#                  "${PROJECT_BINARY_DIR}/../LEti_Engine"
# )
# add_subdirectory("${PROJECT_SOURCE_DIR}/../L_Physics"
#                  "${PROJECT_BINARY_DIR}/../L_Physics"
# )

target_include_directories(L_Physics_Tests PUBLIC
                            "${PROJECT_SOURCE_DIR}"
                            "${PROJECT_SOURCE_DIR}/include"
                            "${PROJECT_SOURCE_DIR}/L_Utility/include"
                            "${PROJECT_SOURCE_DIR}/L_Variable/include"
                            "${PROJECT_SOURCE_DIR}/LEti_Engine/include"
                            "${PROJECT_SOURCE_DIR}/LEti_Engine/OpenGL/GLM"
                            "${PROJECT_SOURCE_DIR}/L_Physics/include"
)


target_link_libraries(L_Physics_Tests PUBLIC
    L_Physics
    LEti_Engine
    L_Variable
    L_Utility
)

if (UNIX)
    target_link_libraries(L_Physics_Tests PUBLIC
        pthread
    )
endif (UNIX)
