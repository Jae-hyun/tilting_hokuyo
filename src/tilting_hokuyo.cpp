#include <ros/ros.h>
#include <dynamixel_msgs/JointState.h>
//#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

#define  PI 3.1415926
#define D2R(degree) ((degree)*(PI/180.0f))
#define R2D(radian) ((radian)*(180.0f/PI))
const float middle_degree = 150;
const float max_degree = 190;
const float min_degree = 130;
float current_pos;
float goal_pos;
void tiltCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("msg.curr_pos(radian): %f, (degree): %f ",msg->current_pos, R2D(msg->current_pos) );
  ROS_INFO("msg.goal_pos(radian): %f, (degree): %f ",msg->goal_pos, R2D(msg->goal_pos) );
  current_pos = R2D(msg->current_pos);
  /*
  static tf::TransformBroadcaster br;
  tf::Transform transform;  
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  transform.setRotation(tf::createQuaternionFromRPY(0.0, (msg->current_pos-middle_degree), 0.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link","tiling_servo"));
  */
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tilting_hokuyo");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("tilt_controller/state", 1, tiltCallback);
  ros::Publisher pub  = nh.advertise<std_msgs::Float64>("tilt_controller/command", 1);
  ros::Rate loop_rate(100);

  std_msgs::Float64 command;
  float cmd_data = middle_degree;
  bool initial_check = false;
  int i = 0;
  while(ros::ok())
  {
    if(!initial_check)
    {
      cmd_data = max_degree;
      ROS_INFO("Initial check, cmd_data(radian): %f, (degree): %f",D2R(cmd_data), cmd_data);
      initial_check = true;
    }
    else
    {
      if(current_pos >= (max_degree-1))
      {
        cmd_data = min_degree;
        ROS_INFO("if max, cmd_data: %f", cmd_data);
      }else if(current_pos <= (min_degree+1))
      {
        cmd_data = max_degree;
        ROS_INFO("if min cmd_data: %f", cmd_data);
      }

      ROS_INFO("titing hokuyo, cmd_data(radian): %f, (degree): %f",D2R(cmd_data), cmd_data);
    }
   // cmd_data -= i;
    i++;
    command.data = D2R(cmd_data);
    pub.publish(command);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
