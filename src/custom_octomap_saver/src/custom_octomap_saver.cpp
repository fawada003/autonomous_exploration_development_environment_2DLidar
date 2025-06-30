#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Trigger.h>
#include <fstream>
#include <tf/tf.h>
#include <opencv2/imgcodecs.hpp>

std::string planner; 
std::string map_topic_name;

class MapSaver
{
public:
  MapSaver(ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh("~");
    pnh.getParam("planner", planner);

    if(planner == "Tare" || planner =="ARiADNE")
      map_topic_name = "projected_map";
    else if(planner == "HPHS")
      map_topic_name = "map";
    else 
      map_topic_name = "projected_map";


    map_sub_    = nh.subscribe(map_topic_name, 1, &MapSaver::mapCallback, this);
    save_srv_   = nh.advertiseService("save_projected_map", &MapSaver::saveService, this);
    has_map_    = false;
  }

private:
  // Subscriber to cache the latest map
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    last_map_ = *msg;
    has_map_ = true;
    //ROS_INFO("Cached new projected_map (size: %u×%u)", msg->info.width, msg->info.height);
  }

  // Service callback to actually write files
  bool saveService(std_srvs::Trigger::Request& req,
                   std_srvs::Trigger::Response& res)
  {
    if (!has_map_)
    {
      res.success = false;
      res.message = "No map received yet, Planner is set to: " + planner;
      return true;
    }

    const auto& msg = last_map_;
    const std::string base = "/home/fawada/autonomous_exploration_development_environment_2DLidar/projected_maps/projected_map";
    cv::Mat image(msg.info.height, msg.info.width, CV_8UC1);

    // fill image (flip vertically so origin ends up bottom-left)
    for (unsigned y = 0; y < msg.info.height; ++y)
      for (unsigned x = 0; x < msg.info.width; ++x)
      {
        int8_t v = msg.data[y * msg.info.width + x];
        uint8_t p = (v == 0 ? 254 : (v == 100 ? 0 : 205));
        image.at<uint8_t>(msg.info.height - y - 1, x) = p;
      }

    // write PGM
    std::string pgm = base + ".pgm";
    if (!cv::imwrite(pgm, image))
    {
      res.success = false;
      res.message = "Failed to write PGM";
      return true;
    }

    // write YAML
    std::string yaml = base + ".yaml";
    std::ofstream out(yaml);
    out << "image: "            << pgm << "\n"
        << "resolution: "       << msg.info.resolution << "\n"
        << "origin: ["
        << msg.info.origin.position.x << ", "
        << msg.info.origin.position.y << ", "
        << tf::getYaw(msg.info.origin.orientation) << "]\n"
        << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    out.close();

    ROS_INFO("Map saved to %s.{pgm,yaml}", base.c_str());
    res.success = true;
    res.message = "Map successfully saved";
    return true;
  }

  ros::Subscriber         map_sub_;
  ros::ServiceServer       save_srv_;
  nav_msgs::OccupancyGrid  last_map_;
  bool                     has_map_;
};

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "projected_map_saver");  
  ros::NodeHandle nh;
  MapSaver saver(nh);
  ROS_INFO("projected_map_saver is ready; call service /save_projected_map to dump the map.");
  ros::spin();
  return 0;
}