#ifndef _COMMAND_HANDLER_H_
#define _COMMAND_HANDLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"


namespace livox_hikcamera_cal
{

    class CommandHandler
    {
        public:
            CommandHandler();
            CommandHandler(ros::NodeHandle node_handle, std::string subscribe_topic, std::string publish_topic);
            void sendCommand(const std::string &content);
            std::string getCommand() const;  
            void resetReceivedStatus();

        private:
            void CommandCallback(const std_msgs::String::ConstPtr& msg);
            ros::NodeHandle nh_;
            ros::Publisher pub_;
            ros::Subscriber sub_;
            std::string latest_command_;  
            bool received_status_;
    };

}



#endif