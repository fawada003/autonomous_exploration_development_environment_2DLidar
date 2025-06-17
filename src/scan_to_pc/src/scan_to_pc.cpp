#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

class ScanToPointCloudPublisher
{
public:
    ScanToPointCloudPublisher(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    {
        // Read parameters
        nh_private.param<std::string>("scan_topic", scan_topic_, "/lidar2D_scan");
        nh_private.param<std::string>("cloud_topic", cloud_topic_, "/fake_points");
        nh_private.param<std::string>("target_frame", target_frame_, ""); // "" = use scan.header.frame_id
        int queue_size;
        nh_private.param<int>("queue_size", queue_size, 10);

        // Create publisher and subscriber
        pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic_, queue_size);
        sub_ = nh.subscribe(scan_topic_, 1, &ScanToPointCloudPublisher::scanCallback, this);

        ROS_INFO_STREAM("[scan_to_pc] Subscribed to " << scan_topic_ << ", will publish PointCloud2 on " << cloud_topic_);
    }

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        sensor_msgs::LaserScan scan_msg = *scan; // Make a copy if we might modify
        if (!target_frame_.empty())
        {
            scan_msg.header.frame_id = target_frame_;
        }

        sensor_msgs::PointCloud2 cloud_msg;
        projector_.projectLaser(scan_msg, cloud_msg);

        pub_.publish(cloud_msg);
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    laser_geometry::LaserProjection projector_;
    std::string scan_topic_;
    std::string cloud_topic_;
    std::string target_frame_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_to_pc");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ScanToPointCloudPublisher node(nh, nh_private);

    ros::spin();
    return 0;
}