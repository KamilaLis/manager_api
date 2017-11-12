#ifndef ALERTMANAGEMENT_H
#define ALERTMANAGEMENT_H

#include <stdlib.h>
//ROS
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

namespace manager_api {

enum Message {killPublisher, killSubsriber, rosTime};
static const std::string EnumStrings[] = { "killPublisher", "killSubsriber", "rosTime" };

const std::string getTextForEnum( int enumVal )
{
  return EnumStrings[enumVal];
}

const Message getEnumForText( std::string text )
{
  int index = std::distance(EnumStrings, std::find(EnumStrings, EnumStrings + sizeof(EnumStrings), text));
  Message msg = static_cast<Message>(index);
  return msg;
}


class AlertManagement
{
public:
    AlertManagement(const std::string& ids_module):
                    ids_module_name_(ids_module)
    {};


    void initPublisher(ros::NodeHandle n)
    {
        diagnostic_pub_ = n.advertise<diagnostic_msgs::DiagnosticStatus>("info", 1);
    }


    // diagnostics
    void sendDiagnosticMsg(const std::string& msg, 
                            int level,
                            const std::string& key = "",
                            const std::string& value = "")
    {
        diagnostic_msgs::DiagnosticStatus message;
        message.level = level;
        message.name = ids_module_name_;
        message.message = msg.c_str();
        if (key!="" && value!="")
        {
            diagnostic_msgs::KeyValue values;
            values.key = key;
            values.value = value;
            message.values = {values};
        }

        diagnostic_pub_.publish(message);
    };

/*
Possible levels of operations
    byte OK=0
    byte WARN=1
    byte ERROR=2
    byte STALE=3



*/

    void warn(const std::string& msg)
    {
        sendDiagnosticMsg(msg, 1);
    };

    void error(const std::string& msg, 
                Message key = rosTime,
                const std::string& value = std::to_string(ros::Time::now().toSec()))
    {
        int n = static_cast<int>(key);
        sendDiagnosticMsg(msg, 2, getTextForEnum(n), value);
    };

    void ok(const std::string& msg)
    {
        sendDiagnosticMsg(msg, 0);
    };

private:
    // publisher
    ros::Publisher diagnostic_pub_;
    std::string ids_module_name_;
};

}; /* namespace */


#endif 
