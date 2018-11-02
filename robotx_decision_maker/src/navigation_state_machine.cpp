#include <navigation_state_machine.h>

navigation_state_machine::navigation_state_machine()
{
    std::string xml_filepath = ros::package::getPath("robotx_decision_maker") + std::string("/data/navigation_state_machine.xml");
    state_machine_ptr_ = std::make_shared<state_machine>(xml_filepath);
    std::string dot_filepath = ros::package::getPath("robotx_decision_maker") + std::string("/data/navigation_state_machine.dot");
    state_machine_ptr_->draw_state_machine(dot_filepath);
}

navigation_state_machine::~navigation_state_machine()
{

}