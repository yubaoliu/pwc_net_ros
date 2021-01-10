#include "action_server.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pwc_net_action_server");
    ros::start();

    pwc_net_ros::ActionServer actionServer;
    if (argc == 2) {
        actionServer.IsSaveResult(true, argv[1]);
    } else {
        actionServer.IsSaveResult(false);
    }

    ros::spin();

    return 0;
}
