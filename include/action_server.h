#ifndef PWC_NET_ACTION_SERVER_NODE_H_
#define PWC_NET_ACTION_SERVER_NODE_H_

#include "pwc_net/pwc_net.h"

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <actionlib/server/simple_action_server.h>
#include <aislam_msg/semanticAction.h>

#include <opencv2/optflow.hpp>

#include <dynamic_reconfigure/server.h>
#include <pwcnet_ros/pwcnetConfig.h>

namespace pwcnet_ros {

class ActionServer {
public:
    ActionServer(ros::NodeHandle& pnh, ros::NodeHandle& gnh);

private:
    // evaluate result
    unsigned long int total_frame_num_;
    float total_estimate_time_;

    // Dynamic reconfigure
    using ReconfigureServer = dynamic_reconfigure::Server<pwcnet_ros::pwcnetConfig>;
    std::shared_ptr<ReconfigureServer> reconfigure_server_;
    ReconfigureServer::CallbackType reconfigure_func_;

    void reconfigureCB(pwcnet_ros::pwcnetConfig& config, uint32_t level);
    void saveResult();

    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Subscriber image_sub_;

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
    std::string save_path_, flow_path_, flow_color_path_;

    // goal
    int id_;
    int command_;

    std::string action_name_;

    std::string flow_pub_topic_;
    ros::Publisher flow_pub_;

    std::string flow_color_pub_topic_;
    ros::Publisher flow_color_pub_;

    ros::NodeHandle gnh_;
    ros::NodeHandle pnh_;
};

} // namespace pwc_net

#endif // PWC_NET__SAMPLE_NODE_H_
