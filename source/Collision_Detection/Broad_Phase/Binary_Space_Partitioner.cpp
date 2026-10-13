#include <Collision_Detection/Broad_Phase/Binary_Space_Partitioner.h>

using namespace LPhys;


void Binary_Space_Partitioner::reset()
{
    m_registered_objects.mark_empty();
    m_registered_borders.mark_empty();
    m_possible_collisions.clear();
}

void Binary_Space_Partitioner::add_models(const Objects_List& _objects)
{
    unsigned int required_size = m_registered_objects.size() + _objects.size();

    if(m_registered_objects.capacity() < required_size)
    {
        m_registered_objects.resize(required_size);
        m_registered_borders.resize(required_size);
    }

    for(Objects_List::Const_Iterator it = _objects.begin(); !it.end_reached(); ++it)
    {
        Physics_Module* module = *it;
        m_registered_objects.push(module);

        LST::Border border;
        module->expand_border(border);
        m_registered_borders.push(border);
    }
}

void Binary_Space_Partitioner::process()
{
    if(m_registered_objects.size() < 2)
        return;

    LST::BVH::Context context;
    context.registered_borders.add(m_registered_borders);

    LST::BVH::ID_Pairs_Vec pairs = m_bvh.process(context);

    for(unsigned int i = 0; i < pairs.size(); ++i)
    {
        const LST::BVH::ID_Pair& pair = pairs[i];

        Physics_Module* module_0 = m_registered_objects[pair.id_0];
        Physics_Module* module_1 = m_registered_objects[pair.id_1];

        if(!module_0->can_collide() || !module_1->can_collide())
            continue;

        if(!passes_filters(module_0, module_1))
            continue;

        m_possible_collisions.push_back({module_0, module_1});
    }
}
