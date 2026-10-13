#include <Collision_Detection/Broad_Phase/Broad_Phase_Interface.h>

using namespace LPhys;


Broad_Phase_Interface::~Broad_Phase_Interface()
{

}



bool Broad_Phase_Interface::passes_filters(const Physics_Module* _first, const Physics_Module* _second) const
{
    for(unsigned int i = 0; i < m_filters.size(); ++i)
    {
        const Filter_Function& filter = m_filters[i];

        if(!filter(_first, _second))
            return false;
    }

    return true;
}
