#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>

#include <erl_map/grid_map.h>
#include <erl_conversions/erl_msg_utils.h>
#include <erl_msgs/GridMap.h>
#include <erl_msgs/GetGridMap.h>


/**
 * The OnlineGridMapServer can read static maps and also interpret ROS occupancy grid maps for online usage. It provides
 * two services
 */
class OnlineGridMapServer
{
  ros::NodeHandle n;
  ros::Subscriber map_sub;
  ros::Publisher map_pub;
  ros::Publisher online_map_pub;
  ros::Publisher lm_pub;
  ros::ServiceServer map_service;
  ros::ServiceServer online_map_service;

  erl_msgs::GetGridMap::Response map_resp_;   // The static map data is cached here, to be sent out to service callers
  erl_msgs::GetGridMap::Response online_map_resp_;   // The online map data is cached here

  // Optional Landmark Information
  erl_msgs::LandmarkMap lm_map;

public:
  OnlineGridMapServer(const std::string& mapfile, const std::string & landmarkfile)
  {
  
    std::string frame_id;
    ros::NodeHandle private_nh("~");
    private_nh.param("frame_id", frame_id, std::string("map"));
    
    
    erl::GridMap<int8_t> mymap;
    ROS_INFO("Loading map from file \"%s\"", mapfile.c_str());
    if( !mymap.load( mapfile ) )
    {
      ROS_ERROR("gridmap_server could not open %s.", mapfile.c_str());
      exit(-1);
    }
    
    map_resp_.map = erl::toROS(mymap);
    map_resp_.map.header.frame_id = frame_id;
    online_map_resp_.map.header.frame_id = frame_id;
    ros::Time::waitForValid();
    map_resp_.map.header.stamp = ros::Time::now();
    
    switch(mymap.dim())
    {
      case 1:
        ROS_INFO("Read a map with %d cells at %.3lf m/cell",
          mymap.size()[0], mymap.res()[0]);
        break;
      case 2:
        ROS_INFO("Read a %d x %d map @ %.3lf x %.3lf m/cell",
          mymap.size()[0], mymap.size()[1],
          mymap.res()[0], mymap.res()[1]);
        break;
      case 3:
        ROS_INFO("Read a %d x %d x %d map @ %.3lf x %.3lf x %.3lf m/cell",
          mymap.size()[0], mymap.size()[1], mymap.size()[2],
          mymap.res()[0], mymap.res()[1], mymap.res()[2]);
        break;
      default:
        ROS_INFO("Read a %d dimensional map", static_cast<int>(mymap.dim()));
    }

    // Parse the Landmark File.
    if (landmarkfile != "") {
      ROS_INFO("Loading landmarks from file \"%s\"", landmarkfile.c_str());
      YAML::Node landmark_file = YAML::LoadFile(landmarkfile);
      lm_map.n_landmarks = landmark_file["n_landmarks"].as<uint32_t>();
      lm_map.landmark_dim = landmark_file["landmark_dim"].as<uint32_t>();
      ROS_INFO("Read in %u landmarks with dimension %u", lm_map.n_landmarks, lm_map.landmark_dim);
      for (const auto & item :  landmark_file["landmark_data"].as<std::vector<std::vector<double>>>()) {
        for (const auto &data : item) {
          lm_map.landmark_data.push_back(data);
        }
      }
      for (const auto &item : landmark_file["landmark_ids"].as<std::vector<uint32_t>>()) {
          lm_map.landmark_ids.push_back(item);
      }
    }

    map_service = n.advertiseService("static_map", &OnlineGridMapServer::mapCallback, this);
    online_map_service = n.advertiseService("online_map", &OnlineGridMapServer::onlineMapCallback, this);

    // Subscriber to Online Map
    map_sub = n.subscribe<nav_msgs::OccupancyGrid>("occupancy_map", 1, &OnlineGridMapServer::occupancyCallback, this);
    ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("occupancy_map", n);

    // Latched publishers
    map_pub = n.advertise<erl_msgs::GridMap>("map", 1, true);
    map_pub.publish( map_resp_.map );

    online_map_pub = n.advertise<erl_msgs::GridMap>("online_map", 1, true);
    online_map_pub.publish( online_map_resp_.map );

    lm_pub = n.advertise<erl_msgs::LandmarkMap>("lm_map", 1, true);
    lm_pub.publish( lm_map);

  }

  /**
   * Callback for incoming occupancy grid messages.
   * @param msg The incoming occupancy grid message.
   */
  void occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    online_map_resp_.map = erl::toROS(erl::fromROS(*msg, true, true));
    online_map_resp_.map.header.frame_id = msg->header.frame_id;
    online_map_resp_.map.header.stamp = ros::Time::now();
    online_map_pub.publish( online_map_resp_.map );
  }

  /** Callback invoked when someone requests our service */
  bool mapCallback(erl_msgs::GetGridMap::Request  &req,
                   erl_msgs::GetGridMap::Response &res )
  {
    // = operator is overloaded to make deep copy (tricky!)
    res = map_resp_;

    if (req.add_landmarks) { // We Add the Landmarks to the Map Request.
      res.lm_map = lm_map;
    }
    ROS_INFO("Sending map");

    return true;
  }

  /** Callback invoked when someone requests our service */
  bool onlineMapCallback(erl_msgs::GetGridMap::Request  &req,
                         erl_msgs::GetGridMap::Response &res )
  {
    // = operator is overloaded to make deep copy (tricky!)
    res = online_map_resp_;

    if (req.add_landmarks) { // We Add the Landmarks to the Map Request.
      res.lm_map = lm_map;
    }
    ROS_INFO("Sending an Online generated map");

    return true;
  }
};


const char* printUsage(){
  return "\nUSAGE: map_server <map.yaml>\n  map.yaml: map description file\n";
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gridmap_server", ros::init_options::AnonymousName);

  if(argc != 2 && argc!= 3)
  {
    ROS_ERROR("%s", printUsage());
    return -1;
  }
  

  std::string mapfile(argv[1]);
  std::string landmarkfile;
  if (argc > 2)
    landmarkfile = argv[2];

  try
  {
    OnlineGridMapServer gms(mapfile, landmarkfile);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("gridmap_server exception: %s", e.what());
    return -1;
  }

  return EXIT_SUCCESS;  
}


