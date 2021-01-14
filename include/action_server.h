#ifndef PWC_NET_ACTION_SERVER_NODE_H_
#define PWC_NET_ACTION_SERVER_NODE_H_

#include "pwc_net/pwc_net.h"

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <actionlib/server/simple_action_server.h>
#include <aislam_msg/semanticAction.h>

#include <opencv2/optflow.hpp>

namespace pwc_net_ros {

class ActionServer {
public:
    ActionServer();
    void IsSaveResult(bool bSave, std::string rootDir = "/root/results/pwcnet");

private:
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Subscriber image_sub_;
    ros::Publisher flow_pub_;
    ros::Publisher visualized_flow_pub_;

    PwcNet pwc_net_;

    // sensor_msgs::ImageConstPtr previous_image_;
    sensor_msgs::ImagePtr previous_image_;

    actionlib::SimpleActionServer<aislam_msg::semanticAction>* as_;

    aislam_msg::semanticFeedback feedback_;
    aislam_msg::semanticResult result_;

    // void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);
    void executeCB(const aislam_msg::semanticGoalConstPtr& t_goal);

    // save results
    bool bIsSaveResult_;
    std::string rootDir_, flow_path_, flow_color_path_;

    // goal
    int id_;
    int command_;

};

} // namespace pwc_net

#endif // PWC_NET__SAMPLE_NODE_H_
