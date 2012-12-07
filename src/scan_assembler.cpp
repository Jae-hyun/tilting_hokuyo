#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>

namespace scan_assembler
{
class ScanAssembler
{
  public:
    ScanAssembler()
    {
      ros::NodeHandle private_nh("~");
      private_nh.param("window", window_, 2.0);

      private_nh.param("publish_rate", publish_rate_, 0.0);

      ros::NodeHandle nh;

      ROS_INFO("Waiting for assemble scan service");
      ros::service::waitForService("assemble_scans");
      ROS_INFO("Subscribing and advertising topics");

      assembler_client_ = nh.serviceClient<laser_assembler::AssembleScans> ("assemble_scans", true);

      laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind(&ScanAssembler::scanCb, this, _1));

      cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("snapshot", 1);

    }

    void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
      if(publish_rate_ == 0.0 || ros::Time::now() - last_pub_ > ros::Duration(1.0/publish_rate_))
      {
        laser_assembler::AssembleScans srv;
        srv.request.begin = scan->header.stamp - ros::Duration(window_);
        srv.request.end = scan->header.stamp;
        assembler_client_.call(srv);

        sensor_msgs::PointCloud2 cloud2;
        //cloud_pub_.publish(srv.response.cloud);
        sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, cloud2);
        cloud_pub_.publish(cloud2);
        
        last_pub_ = ros::Time::now();
      }
    }

  private:
    ros::ServiceClient assembler_client_;
    ros::Subscriber laser_sub_;
    ros::Publisher cloud_pub_;
    double window_;
    ros::Time last_pub_;
    double publish_rate_;
};
}

using namespace scan_assembler;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_assembler");
  ScanAssembler sa;
  ros::spin();
  return 0;
}

