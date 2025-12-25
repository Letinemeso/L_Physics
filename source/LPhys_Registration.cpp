#include <LPhys_Registration.h>

#include <Modules/Physics_Module__Point.h>
#include <Modules/Physics_Module__Ray.h>
#include <Modules/Physics_Module__Mesh.h>

using namespace LPhys;


void LPhys::register_types(LV::Object_Constructor& _object_constructor)
{
    _object_constructor.register_type<LPhys::Physics_Module_Stub__Point>();

    _object_constructor.register_type<LPhys::Physics_Module_Stub__Ray>();

    _object_constructor.register_type<LPhys::Physics_Module_Stub__Mesh>();
}
