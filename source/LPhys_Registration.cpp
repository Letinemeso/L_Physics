#include <LPhys_Registration.h>

#include <Modules/Physics_Module__Point.h>
#include <Modules/Physics_Module__Ray.h>
#include <Modules/Physics_Module_2D.h>
#include <Modules/Rigid_Body_2D.h>

using namespace LPhys;


void LPhys::register_types(LV::Object_Constructor& _object_constructor)
{
    _object_constructor.register_type<LPhys::Physics_Module_Stub__Point>();

    _object_constructor.register_type<LPhys::Physics_Module_Stub__Ray>();

    _object_constructor.register_type<LPhys::Physics_Module_2D_Stub>();

    _object_constructor.register_type<LPhys::Rigid_Body_2D__Stub>();
}
