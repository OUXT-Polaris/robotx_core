#include <mission_state_machine.h>

mission_state_machine::mission_state_machine()
{
    std::string xml_filepath = ros::package::getPath("robotx_decision_maker") + std::string("/data/mission_state_machine.xml");
    std::string dot_filepath = ros::package::getPath("robotx_decision_maker") + std::string("/data/mission_state_machine.dot");
    state_machine_ptr_ = std::make_shared<rostate_machine>(xml_filepath, dot_filepath, "mission_state_machine");
}

mission_state_machine::~mission_state_machine()
{

}

void mission_state_machine::run()
{
    state_machine_ptr_->run();
    return;
}