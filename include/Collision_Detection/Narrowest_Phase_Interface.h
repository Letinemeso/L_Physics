#ifndef __NARROWEST_PHASE_INTERFACE
#define __NARROWEST_PHASE_INTERFACE

#include <Math_Stuff.h>

#include <Collision_Detection/Intersection_Data.h>
#include <Physical_Models/Physical_Model_2D.h>


namespace LPhys
{

    class Narrowest_Phase_Interface
    {
    public:
        Narrowest_Phase_Interface();
        virtual ~Narrowest_Phase_Interface();

    public:
        virtual LEti::Geometry::Simple_Intersection_Data collision__model_vs_point(const Physical_Model_2D& _model, const glm::vec3& _point) const = 0;
        virtual Intersection_Data collision__model_vs_model(const Polygon_Holder_Base* _polygon_holder_1, unsigned int _pols_amount_1, const Polygon_Holder_Base* _polygon_holder_2, unsigned int _pols_amount_2) const = 0;

    };

}


#endif // __NARROWEST_PHASE_INTERFACE
