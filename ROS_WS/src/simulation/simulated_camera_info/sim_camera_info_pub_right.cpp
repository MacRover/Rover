#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <vector>
#include <boost/array.hpp>


const double right_D[] = {-0.00018484236352393622, 0.0003454138376566816, 5.286343471712037e-05, 9.804209121227628e-06, 0.0};
const double right_K[] = {477.1066416484911, 0.0, 399.4558468154678, 0.0, 477.1168907475518, 399.6481701314758, 0.0, 0.0, 1.0305, 0.0, 0.0, 1.0};
const double right_R[] = {0.9999260946349032, 0.0020487032409534063, -0.011983659008046139, -0.00204889063161104, 0.9999979010157414,
                           -3.3601459527441113e-06, 0.01198362697059262, 2.7913104293936906e-05, 0.9999281933746482};
const double right_P[] = {483.1801093735993, 0.0, 409.23957443237305, -33.82747666611492, 0.0, 483.1801093735993, 399.6501274108887, 0.0, 0.0, 0.0, 1.0, 0.0};

ros::Publisher pubRight;



void msgCallbackRight(const sensor_msgs::CameraInfo& msg)
{
  sensor_msgs::CameraInfo copy;
  copy.header = msg.header;
  copy.height = msg.height;
  copy.width = msg.width;
  copy.distortion_model = msg.distortion_model;
  copy.D = std::vector<double>(std::begin(right_D), std::end(right_D));
  std::copy(std::begin(right_K), std::end(right_K), std::begin(copy.K));
  std::copy(std::begin(right_R), std::end(right_R), std::begin(copy.R));
  std::copy(std::begin(right_P), std::end(right_P), std::begin(copy.P));
  pubRight.publish(copy);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "sim_camera_info_pub_right");
  ros::NodeHandle nh;
  ros::Subscriber sub2 = nh.subscribe("multisense_sl/camera/right/camera_info", 100, msgCallbackRight);
  pubRight = nh.advertise<sensor_msgs::CameraInfo>("simulation/right/camera_info",100);
  ros::spin();
  return 0;
}