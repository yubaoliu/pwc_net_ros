#ifndef PWC_NET_ACTION_SERVER_NODE_H_
#define PWC_NET_ACTION_SERVER_NODE_H_

#include "pwc_net/pwc_net.h"

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <actionlib/server/simple_action_server.h>
#include <pwc_net/opticalflowAction.h>

namespace pwc_net {

class ActionServer {
private:
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Subscriber image_sub_;
    ros::Publisher flow_pub_;
    ros::Publisher visualized_flow_pub_;

    PwcNet pwc_net_;

    // sensor_msgs::ImageConstPtr previous_image_;
    sensor_msgs::ImagePtr previous_image_;

    actionlib::SimpleActionServer<pwc_net::opticalflowAction>* as_;

    opticalflowFeedback feedback_;
    opticalflowResult result_;

    // void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);
    void executeCB(const opticalflowGoalConstPtr& t_goal);

public:
    ActionServer();
};

} // namespace pwc_net

#endif // PWC_NET__SAMPLE_NODE_H_