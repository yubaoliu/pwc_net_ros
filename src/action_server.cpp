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

using namespace aislam_msg;
namespace pwc_net_ros {
ActionServer::ActionServer()
{
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");

    bIsSaveResult_ = false;
    command_ = 0;

    std::string action_name;
    node_handle.getParam("action_name", action_name);

    flow_pub_ = private_node_handle.advertise<sensor_msgs::Image>("/optical_flow", 5);
    visualized_flow_pub_ = private_node_handle.advertise<sensor_msgs::Image>("/visualized_optical_flow", 5);

    as_ = new actionlib::SimpleActionServer<aislam_msg::semanticAction>(node_handle, action_name, boost::bind(&ActionServer::executeCB, this, _1), false);
    as_->start();

    ROS_INFO("action name: %s \n", action_name);
}

void ActionServer::executeCB(const semanticGoalConstPtr& goal)
{
    if (!as_->isActive()) {
        ROS_ERROR("action server starting failed");
        return;
    }
    id_ = goal->id;
    ROS_INFO("-------ID: %d ------------", id_);
    command_ = goal->command;
    if (command_ == 1) {
        ROS_INFO("Reset the loop");
        previous_image_.reset(new sensor_msgs::Image);
    }

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
            result_.id = goal->id;
            result_.mask= *opticalflowMsg;
            as_->setSucceeded(result_);

            // publish the results
            if (flow_pub_.getNumSubscribers() > 0) {
                flow_pub_.publish(opticalflowMsg);
            }

            // TODO use a thread to deal with
            cv_bridge::CvImage visualized_optical_flow(image->header, sensor_msgs::image_encodings::BGR8);
            pwc_net_.visualizeOpticalFlow(optical_flow.image, visualized_optical_flow.image, 5.0f);
            if (visualized_flow_pub_.getNumSubscribers()) {
                visualized_flow_pub_.publish(visualized_optical_flow.toImageMsg());
            }
            if (bIsSaveResult_) {
                std::string name = flow_path_ + std::to_string(goal->id) + ".png";
                cv::optflow::writeOpticalFlow(name, optical_flow.image);

                name = flow_color_path_ + std::to_string(goal->id) + ".png";
                cv::imwrite(name, visualized_optical_flow.image);
            }
        }
    }

    previous_image_ = image;
}

void ActionServer::IsSaveResult(bool bSave, std::string rootDir)
{
    bIsSaveResult_ = bSave;
    rootDir_ = rootDir;

    // LOG(INFO) << "Root directory: " << rootDir_;
    ROS_INFO("Root directory: %s", rootDir_);

    std::string str = rootDir;
    system(("mkdir -p " + str).c_str());

    flow_path_ = rootDir + "/flow/";
    system(("mkdir -p " + flow_path_).c_str());

    flow_color_path_ = rootDir + "/flow_color/";
    system(("mkdir -p " + flow_color_path_).c_str());
}
}
