#include "action_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pwc_net_action_server");
  pwc_net::ActionServer actionServer;
  ros::spin();

  return 0;
}
