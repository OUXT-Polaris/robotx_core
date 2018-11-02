#include <navigation_state_machine.h>

navigation_state_machine::navigation_state_machine()
{
    std::string xml_filepath = ros::package::getPath("robotx_decision_maker") + std::string("/data/navigation_state_machine.xml");
    std::string dot_filepath = ros::package::getPath("robotx_decision_maker") + std::string("/data/navigation_state_machine.dot");
    state_machine_ptr_ = std::make_shared<rostate_machine>(xml_filepath, dot_filepath);
}

navigation_state_machine::~navigation_state_machine()
{

}