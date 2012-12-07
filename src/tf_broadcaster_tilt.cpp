#include <ros/ros.h>
#include <dynamixel_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

#define  PI 3.1415926
#define D2R(degree) ((degree)*(PI/180.0f))
#define R2D(radian) ((radian)*(180.0f/PI))
const float middle_degree = 150;
const float max_degree = 180;
const float min_degree = 120;

void tiltCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;  
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0425));
  transform.setRotation(tf::createQuaternionFromRPY(0.0, (middle_degree-msg->current_pos), 0.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link","tilt_base"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_broadcaster_tilt");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("tilt_controller/state", 1, tiltCallback);

  ros::spin();

  return 0;

}
