#include "action_server.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <vector>

namespace pwc_net {
ActionServer::ActionServer()
{
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");

    std::string action_name;
    node_handle.getParam("action_name", action_name);

    flow_pub_ = private_node_handle.advertise<sensor_msgs::Image>("optical_flow", 1);
    visualized_flow_pub_ = private_node_handle.advertise<sensor_msgs::Image>("visualized_optical_flow", 1);

    as_ = new actionlib::SimpleActionServer<pwc_net::opticalflowAction>(node_handle, action_name, boost::bind(&ActionServer::executeCB, this, _1), false);
    as_->start();

    ROS_INFO("action name: %s \n", action_name);
}

void ActionServer::executeCB(const opticalflowGoalConstPtr& goal)
{
    if (!as_->isActive()) {
        ROS_ERROR("action server starting failed");
        return;
    }
    std::cout << "id: " << goal->id << std::endl;
    ROS_INFO("id: %d", goal->id);

    cv_bridge::CvImageConstPtr cvpImage;
    try {
        cvpImage = cv_bridge::toCvCopy(goal->image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        feedback_.complete = false;
        as_->publishFeedback(feedback_);
        return;
    }
    sensor_msgs::ImagePtr image = cvpImage->toImageMsg();
    if (previous_image_) {
        cv_bridge::CvImage optical_flow(cvpImage->header, sensor_msgs::image_encodings::TYPE_32FC2);
        bool success = pwc_net_.estimateOpticalFlow(*previous_image_, *image, optical_flow.image);

        if (success) {
            sensor_msgs::ImagePtr opticalflowMsg = optical_flow.toImageMsg();
            flow_pub_.publish(opticalflowMsg);
            result_.id = goal->id;
            result_.opticalflow = *opticalflowMsg;
            as_->setSucceeded(result_);

            cv_bridge::CvImage visualized_optical_flow(image->header, sensor_msgs::image_encodings::BGR8);
            pwc_net_.visualizeOpticalFlow(optical_flow.image, visualized_optical_flow.image, 5.0f);

            visualized_flow_pub_.publish(visualized_optical_flow.toImageMsg());
        }
    }

    previous_image_ = image;
}
}
