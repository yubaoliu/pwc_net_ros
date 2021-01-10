
#include "action_client.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pwc_net_action_server");
    pwc_net_ros::ActionClient actionClient;

    ros::spin();

    return 0;
}
