#include <Collision_Detection/Broad_Phase/Broad_Phase_Interface.h>

using namespace LPhys;


Broad_Phase_Interface::~Broad_Phase_Interface()
{

}



bool Broad_Phase_Interface::passes_filters(const Physics_Module* _first, const Physics_Module* _second) const
{
    for(Filters_List::Const_Iterator filter_it = m_filters.begin(); !filter_it.end_reached(); ++filter_it)
    {
        const Filter_Function& filter = *filter_it;

        if(!filter(_first, _second))
            return false;
    }

    return true;
}
