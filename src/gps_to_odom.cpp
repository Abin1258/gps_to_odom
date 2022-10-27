#include <gps_to_odom/gps_to_odom.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_to_odom");

  ros::NodeHandle nh;

  GpsOdom *gps_to_odom = new GpsOdom(nh);
  
  ros::spin();

  return 0;
}
