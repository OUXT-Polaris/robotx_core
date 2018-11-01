#include <state_machine.h>

state_machine::state_machine()
{

}

state_machine::~state_machine()
{
    
}

bool state_machine::set_current_state(std::string current_state)
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto vertex_range = boost::vertices(state_graph_);
    for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first)
    {
        vertex_t v = *first;
        if(state_graph_[v].name == current_state)
        {
            current_state_ = v;
            return true;
        }
    }
    return false;
}

void state_machine::add_transition(std::string from_state_name, std::string to_state_name, std::string trigger_event_name)
{
    std::lock_guard<std::mutex> lock(mtx_);
    vertex_t from_state;
    vertex_t to_state;
    edge_t transition;
    bool from_state_found = false;
    bool to_state_found = false;
    bool edge_found = false;
    auto vertex_range = boost::vertices(state_graph_);
    for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first)
    {
        vertex_t v = *first;
        if(state_graph_[v].name == from_state_name)
        {
            from_state_found = true;
            from_state = v;
        }
        if(state_graph_[v].name == to_state_name)
        {
            to_state_found = true;
            to_state = v;
        }
    }
    if(!from_state_found)
    {
        vertex_t v = boost::add_vertex(state_graph_);
        state_graph_[v].name = from_state_name;
        from_state = v;
    }
    if(!to_state_found)
    {
        vertex_t v = boost::add_vertex(state_graph_);
        state_graph_[v].name = to_state_name;
        to_state = v;
    }
    bool inserted = false;
    boost::tie(transition, inserted) = boost::add_edge(from_state, to_state, state_graph_);
    state_graph_[transition].trigger_events.push_back(trigger_event_name);
    return;
}

std::vector<std::string> state_machine::get_possibe_transition_states()
{
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::string> ret;
    adjacency_iterator_t vi;
    adjacency_iterator_t vi_end;
    for (boost::tie(vi, vi_end) = adjacent_vertices(current_state_, state_graph_); vi != vi_end; ++vi)
    {
        ret.push_back(state_graph_[*vi].name);
    }
    return ret;
}