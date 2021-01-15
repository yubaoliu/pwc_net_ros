#include "action_server.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pwcnet_action_server");
    ros::start();

    ros::NodeHandle gnh;
    ros::NodeHandle pnh("~");

    pwcnet_ros::ActionServer actionServer(pnh, gnh);

    // if (argc == 2) {
    //     actionServer.IsSaveResult(true, argv[1]);
    // } else {
    //     actionServer.IsSaveResult(false);
    // }

    ros::spin();

    return 0;
}
