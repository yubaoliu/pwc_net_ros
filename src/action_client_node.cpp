
#include "action_client.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pwcnet_action_server");
    pwcnet_ros::ActionClient actionClient;

    ros::spin();

    return 0;
}
