#ifndef PWC_NET__ACTION_CLIENT_NODE_H_
#define PWC_NET__ACTION_CLIENT_NODE_H_

#include "pwc_net/pwc_net.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <actionlib/client/simple_action_client.h>
#include <pwc_net/opticalflowAction.h>

namespace pwc_net {
class ActionClient {
public:
    ActionClient();
    void imageCallback(const sensor_msgs::ImageConstPtr& image);

    bool estimateOpticalFlow(
        const sensor_msgs::Image& source_image_msg,
        cv::Mat& optical_flow);
    void visualize(
        const cv::Mat& optical_flow,
        cv::Mat& visualized_optical_flow,
        float max_magnitude);

private:
    actionlib::SimpleActionClient<pwc_net::opticalflowAction>* ac_;
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Subscriber image_sub_;
    ros::Publisher flow_pub_;

    opticalflowResultConstPtr result_;
    long int factory_id_;
};
}

#endif
