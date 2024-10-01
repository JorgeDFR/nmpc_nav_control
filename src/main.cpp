#include "nmpc_nav_control/NMPCNavControlROS.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "nmpc_nav_control");

  nmpc_nav_control::NMPCNavControlROS node;
  
  ros::shutdown();

  return 0;
}