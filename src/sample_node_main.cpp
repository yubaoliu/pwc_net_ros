#include "sample_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pwc_net_sample");
  pwcnet_ros::SampleNode sample_node;
  ros::spin();

  return 0;
}
