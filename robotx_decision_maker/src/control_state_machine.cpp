#include <control_state_machine.h>

control_state_machine::control_state_machine()
{
    state_machine_ptr_ = std::make_shared<state_machine>();
}

control_state_machine::~control_state_machine()
{

}