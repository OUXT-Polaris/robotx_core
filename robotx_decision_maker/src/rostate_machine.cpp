#include <rostate_machine.h>

rostate_machine::rostate_machine(std::string xml_filepath, std::string dot_filepath, std::string state_machine_name)
{
    state_machine_ptr_ = std::make_shared<state_machine>(xml_filepath);
    state_machine_ptr_->draw_state_machine(dot_filepath);
    state_machine_name_ = state_machine_name;
}

rostate_machine::~rostate_machine()
{

}