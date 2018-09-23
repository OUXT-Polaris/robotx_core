#include <robotx_debag_tools/robotx_glog_tools.h>

robotx_glog_tool::robotx_glog_tool()
{
    google::InitGoogleLogging("");
    google::InstallFailureSignalHandler();
}

robotx_glog_tool::~robotx_glog_tool()
{

}