#include "action_client.h"
using namespace aislam_msg;
namespace pwcnet_ros {
ActionClient::ActionClient()
{
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");

    ROS_INFO("Waiting for action server to start.");
    std::cout << "Waiting for action server to start." << std::endl;
    factory_id_ = -1;

    std::string image_topic = node_handle.resolveName("image");
    // std::string action_name = node_handle.resolveName("action_name");
    std::string action_name;
    node_handle.getParam("action_name", action_name);

    ac_ = new actionlib::SimpleActionClient<aislam_msg::semanticAction>(node_handle, action_name, true);
    ac_->waitForServer();

    image_transport_.reset(new image_transport::ImageTransport(node_handle));
    image_sub_ = image_transport_->subscribe(image_topic, 1, &ActionClient::imageCallback, this);

    flow_pub_ = private_node_handle.advertise<sensor_msgs::Image>("optical_flow", 1);

    ROS_INFO("action name: %s \n", action_name);
    ROS_INFO("~~~Action server started, sending goal.~~~~~");
}

void ActionClient::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    ROS_INFO("------------------------------------------");
    cv_bridge::CvImage optical_flow(image->header, sensor_msgs::image_encodings::TYPE_32FC2);
    bool success = estimateOpticalFlow(*image, optical_flow.image);

    // if (success) {
    //     if (flow_pub_.Publisher::getNumSubscribers() > 1) {
    //         flow_pub_.publish(optical_flow.toImageMsg());
    //     }
    // }
}

bool ActionClient::estimateOpticalFlow(
    const sensor_msgs::Image& source_image_msg,
    cv::Mat& optical_flow)
{
    semanticGoal goal;
    goal.id = ++factory_id_;
    goal.image = source_image_msg;

    // Need boost::bind to pass in the 'this' pointer
    ac_->sendGoal(goal);

    std::cout << "Request ID: " << goal.id << std::endl;

    //wait for the action to return
    bool finished_before_timeout = ac_->waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac_->getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());

        result_ = ac_->getResult();
        std::cout << "Received ID: " << result_->id << std::endl;

        // visualize result
        cv_bridge::CvImageConstPtr cvpImage;
        try {
            cv::Mat visualized_optical_flow;
            cvpImage = cv_bridge::toCvCopy(result_->mask, "32FC2");
            visualize(cvpImage->image, visualized_optical_flow, 5.0f);
            cv::imshow("opticalflow", visualized_optical_flow);
            cv::waitKey(1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

    } else {
        ROS_INFO("Action did not finish before the time out.");
        return false;
    }
}

void ActionClient::visualize(
    const cv::Mat& optical_flow,
    cv::Mat& visualized_optical_flow,
    float max_magnitude)
{
    cv::Mat hsv_image(optical_flow.rows, optical_flow.cols, CV_8UC3, cv::Vec3b(0, 0, 0));

    int total_pixels = optical_flow.total();
    for (int i = 0; i < total_pixels; i++) {
        const cv::Vec2f& flow_at_point = optical_flow.at<cv::Vec2f>(i);

        float flow_magnitude = std::sqrt(flow_at_point[0] * flow_at_point[0] + flow_at_point[1] * flow_at_point[1]);
        float flow_direction = std::atan2(flow_at_point[0], flow_at_point[1]);

        uchar hue = (flow_direction / M_PI + 1.0) / 2.0 * 255;
        uchar saturation = std::min(std::max(flow_magnitude / max_magnitude, 0.0f), 1.0f) * 255;
        uchar value = 255;

        cv::Vec3b& hsv = hsv_image.at<cv::Vec3b>(i);
        hsv[0] = hue;
        hsv[1] = saturation;
        hsv[2] = value;
    }

    cv::cvtColor(hsv_image, visualized_optical_flow, cv::ColorConversionCodes::COLOR_HSV2BGR_FULL);
}
}
