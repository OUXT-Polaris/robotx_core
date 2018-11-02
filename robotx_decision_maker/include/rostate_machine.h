#ifndef ROSTATE_MACHINE_H_INCLUDED
#define ROSTATE_MACHINE_H_INCLUDED

#include "state_machine.h"

//headers in STL
#include <memory>

class rostate_machine
{
public:
    rostate_machine(std::string xml_filepath, std::string dot_filepath);
    ~rostate_machine();
private:
    std::shared_ptr<state_machine> state_machine_ptr_;
};

#endif  //ROSTATE_MACHINE_H_INCLUDED