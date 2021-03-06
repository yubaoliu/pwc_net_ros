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
namespace pwcnet_ros {
ActionServer::ActionServer(ros::NodeHandle& pnh, ros::NodeHandle& gnh)
    : pnh_(pnh)
    , gnh_(gnh)
{
    // Dynamic reconfigure
    reconfigure_server_.reset(new ReconfigureServer(pnh_));
    reconfigure_func_ = boost::bind(&ActionServer::reconfigureCB, this, _1, _2);
    reconfigure_server_->setCallback(reconfigure_func_);
    previous_image_.reset(new sensor_msgs::Image);

    command_ = 0;

    flow_pub_ = pnh_.advertise<sensor_msgs::Image>(flow_pub_topic_, 5);
    flow_color_pub_ = pnh_.advertise<sensor_msgs::Image>(flow_color_pub_topic_, 5);

    as_ = new actionlib::SimpleActionServer<aislam_msg::semanticAction>(gnh_, action_name_, boost::bind(&ActionServer::executeCB, this, _1), false);
    as_->start();

    this->saveResult();
    ROS_INFO("---- PWCNet Ready -----");
}

void ActionServer::reconfigureCB(pwcnet_ros::pwcnetConfig& config, uint32_t level)
{
    ROS_INFO("Reconfigure Request");
    bIsSaveResult_ = config.bIsSaveResult;
    save_path_ = config.save_path;
    action_name_ = config.action_name;
    flow_pub_topic_ = config.flow_pub_topic;
    flow_color_pub_topic_ = config.flow_color_pub_topic;

    ROS_INFO("action name: %s \n", action_name_);
}

void ActionServer::executeCB(const semanticGoalConstPtr& goal)
{
    if (!as_->isActive()) {
        ROS_ERROR("action server starting failed");
        return;
    }
    ROS_INFO("-------ID: %d ------------", id_);
    id_ = goal->id;
    command_ = goal->command;

    if ((command_ == 1) || (id_ == 0)) {
        ROS_INFO("Reset the loop");
        previous_image_.reset(new sensor_msgs::Image);
        // for evaluation
        total_frame_num_ = 0;
        total_estimate_time_ = 0;
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
    cv_bridge::CvImagePtr optical_flow(new cv_bridge::CvImage(cvpImage->header, sensor_msgs::image_encodings::TYPE_32FC2));
    if (previous_image_->width > 1) {
        // estimate Optical flow
        ros::WallTime start_process = ros::WallTime::now();
        // TODO backward flow
        // bool success = pwc_net_.estimateOpticalFlow(*previous_image_, *image, optical_flow->image);
        // TODO forward flow, check the reason later
        bool success = pwc_net_.estimateOpticalFlow(*image, *previous_image_, optical_flow->image);
        ros::WallDuration process_time = ros::WallTime::now() - start_process;
        ROS_INFO("process time: %f ms", process_time.toSec() * 1000);
        total_frame_num_++;
        total_estimate_time_ += process_time.toSec();
        if (total_frame_num_ > 20)
            ROS_INFO("Average process time: %f ms", total_estimate_time_ / total_frame_num_ * 1000);

        if (success) {
            sensor_msgs::ImagePtr opticalflowMsg = optical_flow->toImageMsg();
            result_.id = goal->id;
            result_.mask = *opticalflowMsg;
            as_->setSucceeded(result_);

            // publish the results
            if (flow_pub_.getNumSubscribers() > 0) {
                flow_pub_.publish(opticalflowMsg);
            }

            // TODO use a thread to deal with
            cv_bridge::CvImagePtr visualized_optical_flow(new cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::BGR8));
            pwc_net_.visualizeOpticalFlow(optical_flow->image, visualized_optical_flow->image, 5.0f);
            if (flow_color_pub_.getNumSubscribers()) {
                flow_color_pub_.publish(visualized_optical_flow->toImageMsg());
            }
            if (bIsSaveResult_) {
                std::string name = flow_path_ + std::to_string(goal->id) + ".flo";
                cv::optflow::writeOpticalFlow(name, optical_flow->image);

                name = flow_color_path_ + std::to_string(goal->id) + ".png";
                cv::imwrite(name, visualized_optical_flow->image);
            }
        }
    }

    previous_image_ = image;
}

void ActionServer::saveResult()
{
    if (bIsSaveResult_) {
        // LOG(INFO) << "Root directory: " << save_path_;
        ROS_INFO("Root directory: %s", save_path_);

        std::string str = save_path_;
        system(("mkdir -p " + str).c_str());

        flow_path_ = save_path_ + "/flow/";
        system(("mkdir -p " + flow_path_).c_str());

        flow_color_path_ = save_path_ + "/flow_color/";
        system(("mkdir -p " + flow_color_path_).c_str());
    }
}
}
