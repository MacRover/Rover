#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <stdio.h>

#define RAD2DEG(x) ((x)*180./M_PI)

using namespace ROS;

void listener_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  int count = scan->scan_time/scan->time_increment;
  for(int i = 0; i < count; i++) {
      float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
      if(degree > -5 && degree< 5){
        printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);
      }
  }
  // scan->.frame_id.c_str() for laser info (gets distance)
  // scan->angle_min , scan->angle_max returns angle in radians

}

int main(int argc, char const *argv[]) {
  /* code */
  init(argc, argv, "custom_linar_node");
  NodeHandle node;
  Subscriber subscriber = node.subscribe<sensor_msgs::LaserScan>("/scan   ", 1000, listener_callback);
  spin();
  return 0;
}
