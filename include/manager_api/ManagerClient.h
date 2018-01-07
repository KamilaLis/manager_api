#ifndef MANAGERCLIENT_H
#define MANAGERCLIENT_H

#include <stdlib.h>
//ROS
#include <ros/ros.h>
#include "manager_api/Manager.h"

namespace manager_api {

class ManagerClient
{
public:
    ManagerClient(const std::string& ids_module):
                    ids_module_name_(ids_module)
    {};

    void initManagerClient(ros::NodeHandle n)
    {
        client_ = n.serviceClient<manager_api::Manager>("manager");
    }

    bool sendDiagnosticMsg(const std::string& msg, 
                            int level,
                            const std::string& request = "",
                            const std::string& topic = "")
    {
        manager_api::Manager srv;
        srv.request.status.level = level;
        srv.request.status.name = ids_module_name_;
        srv.request.status.message = msg.c_str();
        if (request!="" && topic!=""){
            diagnostic_msgs::KeyValue values;
            values.key = request;
            values.value = topic;                  
            srv.request.status.values = {values};
        }
        if (client_.call(srv)){
            ROS_INFO("Done: %d", (int)srv.response.done);
        }
        else{
            ROS_ERROR("Failed to call service manager.");
            return false;
        }
        return true;
    };

/* ===============================================
        Possible levels of operations
            byte OK=0
            byte WARN=1
            byte ERROR=2
            byte STALE=3
  ================================================  */

    bool warn(const std::string& msg)
    {
        return sendDiagnosticMsg(msg, 1);
    };

    bool error(const std::string& msg, 
                Message request,
                const std::string& topic)
    {
        int n = static_cast<int>(request);
        return sendDiagnosticMsg(msg, 2, getTextForEnum(n), topic);
    };

    bool ok(const std::string& msg)
    {
        return sendDiagnosticMsg(msg, 0);
    };

private:
    ros::ServiceClient client_;
    std::string ids_module_name_;
};

}; /* namespace */


#endif 
