TEMPLATE = lib
CONFIG += staticlib
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += "../LEti_Engine/OpenGL/GLM"
INCLUDEPATH += "include/"

INCLUDEPATH += "../L_Variable/include/"
INCLUDEPATH += "../L_Utility/include/"
INCLUDEPATH += "../LEti_Engine/include/"
#   temporary stuff
INCLUDEPATH += "../L_Renderer/include/"
unix {
    INCLUDEPATH += "../LEti_Engine/OpenGL/Linux/include/"
}


win32 {
    INCLUDEPATH += "../LEti_Engine/OpenGL/Windows_x64_mingw/include/"
}

HEADERS += \
    include/Modules/Dynamic_Physics_Module_2D.h \
    include/Modules/Physics_Module_Base.h \
    include/Modules/Physics_Module__Rigid_Body_2D.h \
    include/Physical_Models/Physical_Model_2D.h \
    include/Physical_Models/Polygon.h \
    include/Physical_Models/Rigid_Body_Physical_Model_2D.h \
    include/Physical_Models/Rigid_Body_Polygon.h \
    include/Collision_Resolution/Collision_Resolver.h \
    include/Collision_Resolution/Collision_Resolution__Rigid_Body_2D.h \
    include/Collision_Detection/Intersection_Data.h \
    include/Collision_Detection/Broad_Phase_Interface.h \
    include/Collision_Detection/Collision_Detector_2D.h \
    include/Collision_Detection/Default_Narrowest_CD.h \
    include/Collision_Detection/Dynamic_Narrow_CD.h \
    include/Collision_Detection/Narrow_Phase_Interface.h \
    include/Collision_Detection/Narrowest_Phase_Interface.h \
    include/Collision_Detection/SAT_Narrowest_CD.h \
    include/Collision_Detection/Space_Hasher_2D.h \

SOURCES += \
    source/Modules/Dynamic_Physics_Module_2D.cpp \
    source/Modules/Physics_Module_Base.cpp \
    source/Modules/Physics_Module__Rigid_Body_2D.cpp \
    source/Physical_Models/Physical_Model_2D.cpp \
    source/Physical_Models/Polygon.cpp \
    source/Physical_Models/Rigid_Body_Physical_Model_2D.cpp \
    source/Physical_Models/Rigid_Body_Polygon.cpp \
    source/Collision_Resolution/Collision_Resolver.cpp \
    source/Collision_Resolution/Collision_Resolution__Rigid_Body_2D.cpp \
    source/Collision_Detection/Intersection_Data.cpp \
    source/Collision_Detection/Broad_Phase_Interface.cpp \
    source/Collision_Detection/Collision_Detector_2D.cpp \
    source/Collision_Detection/Default_Narrowest_CD.cpp \
    source/Collision_Detection/Dynamic_Narrow_CD.cpp \
    source/Collision_Detection/Narrow_Phase_Interface.cpp \
    source/Collision_Detection/Narrowest_Phase_Interface.cpp \
    source/Collision_Detection/SAT_Narrowest_CD.cpp \
    source/Collision_Detection/Space_Hasher_2D.cpp \
