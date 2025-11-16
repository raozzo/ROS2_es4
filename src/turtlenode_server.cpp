#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>
#include <cmath>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "turtle_interfaces/srv/find_apples.hpp"

class TurtlenodeServer : public rclcpp::Node
{
  public:
  using FindApples = turtle_interfaces::srv::FindApples;
    
  //constructor
  TurtlenodeServer():Node("turtlenode_server")
  {
    
    this->srv_ = this->create_service<FindApples>(
        "find_apples",
        [this](const std::shared_ptr<FindApples::Request> request,
               std::shared_ptr<FindApples::Response> response) {
            this->find_apples_callback(request, response);
        });

    this->scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            this->scan_callback(msg);
        })
    // Initialize the apple count
    this->visible_apples_ = 0;
        
    RCLCPP_INFO(this->get_logger(), "Server is ready. Waiting for requests.");
  }

  private:
  struct Point 
  {
    double x;
    double y;
  };

  struct ApplePosition {
    double x, y;
    bool success;
  };

  //to find the position i keep the closer point to the robot 
  // an alternative could be to use the mid point of the chord as an approximated center
  ApplePosition find_apple_position(const std::vector<Point2D>& cluster)
{
    if (cluster.empty()) {
        return {0, 0, false};
    }

    double min_range_sq = std::numeric_limits<double>::max();
    Point2D closest_point = {0, 0};

    // Iterate through all points in this one cluster
    for (const auto& point : cluster)
    {
        // Calculate squared range (faster than sqrt)
        double range_sq = point.x * point.x + point.y * point.y;
        
        if (range_sq < min_range_sq)
        {
            min_range_sq = range_sq;
            closest_point = point;
        }
    }
    return {closest_point.x, closest_point.y, true};
}

  //the idea  is that it's an arc theac mid point should be closer to the turtlebot than the "corda"
  bool is_arc(const std::vector<Point>& cluster, double threshold)
  {
    if (cluster.size<3) {
      //not enough point
      return false;  
    }
    
    Point start_point = cluster.front();
    Point end_point = cluster.back(); 
    Point mid_point = cluster[cluster.size()/2]; 

    Point corda_midpoint = 
    {
      (start_point.x + end_point.x)/2,
      (start_point.y + end_point.y)/2,
    };
    double mid_point_range = std::sqrt(mid_point.x * mid_point.x + mid_point.y * mid_point.y);
    double chord_midpoint_range = std::sqrt(corda_midpoint.x * corda_midpoint.x + corda_midpoint.y * corda_midpoint.y);
    return (corda_midpoint_range - mid_point_range) > convexity_threshold;
  }
  
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    //lodar logic
    //the idea is ""cluster"" point provided by the lidar and then 
    //count only those cluster that resemble an arc in 2D space (->sphere in 3d space)
    //
    //TODO
    //1. convert in cartesian coordinates
    //2. cluster (i will try to cluster using only distance)
    //3. select cluster based on number of points / size / by curvature
    //
    //
    const double CLUSTER_THRESHOLD = 0.15; 
    const int MIN_CLUSTER_POINTS = 3;
    const int MAX_CLUSTER_POINTS = 15;
    const double MAX_APPLE_SPAN = 0.3;       // 30cm
    const double CONVEXITY_THRESHOLD = 0.01; // 1cm

    //vector of  filtered cluster (only those cluster that have enough number of points and not too much)
    std::vector<std::vector<Point>> filtered_cluster;

    //hold points of current cluster
    std::vector<Point> current_cluster;

    //to save te previous point 
    Point last_point = {0,0};

    //now i have to compute all the points i receive from laserscan
    for (size_t i = 0; i< msg->ranges.size(); i++)
    {
      float range = msg -> ranges[i];

      //cartesian conversion
      //validity check on received ranges
      if (range < msg->range_min || range > msg->range_max || std::isinf(range))
      {
        //the idea is that i receive an invalid point and i'm in a cluster i save it, else i continue 

        if(!current_cluster.empty())
        {
          if(current_cluster.size()>= MIN_CLUSTER_POINTS && current_cluster.size()<= MAX_CLUSTER_POINTS)
          {
            //save the cluster in the vector of clusters 
            filtered_cluster.push_back(current_cluster);
          }

          //if not a valid size
          current_cluster.clear(); 
        }

        continue;
      }
      
      //conversion in cartesian
      //i have to get the angle usign the starting angle and current iteration
      double angle = msg -> angle_min + (i*msg->angle_increment); 

      Point current_point = {
        range* std::cos(angle),
        range*std::sin(angle)
      };

      //CLUSTERIGN
      if(current_cluster.empty())
      {
        current_cluster.push_back(current_point);
        last_point = current_point;
        continue; 
      }

      //diatance from last point
      double dist_x = current_point.x - last_point.x;
      double dist_y = current_point.y - last_point.y;

      double distance = std::sqrt(pow(dist_x,2)+pow(dist_y,2));
      
      //if the distance is small enough i add to the cluster else i end the previous one and start a new one
      if (distance < CLUSTER_THRESHOLD) 
      {
        current_cluster.push_back(current_point);
      }
      else
      {
        if(current_cluster.size()>= MIN_CLUSTER_POINTS && current_cluster.size()<= MAX_CLUSTER_POINTS)
          {
            //save the cluster in the vector of clusters 
            filtered_cluster.push_back(current_cluster);
          }

          //if not a valid size
          current_cluster.clear();
          //add to a new cluster
          current_cluster.push_back(current_point);
      }

      last_point=current_point;
    }


    if (!current_cluster.empty()) 
    {
      if (current_cluster.size() >= MIN_CLUSTER_POINTS && current_cluster.size() <= MAX_CLUSTER_POINTS) 
      {
        filtered_clusters.push_back(current_cluster);
      }
    }
    
    //now that i have an array of fitered clusters
  
    //1. I have to decide if it's an apple 
    //2. have to compute the midpoint to send it to the pose message 
    
    int apple_count = 0; 
    auto pose_array_msg = std::make_unique<geometry_msgs::msg::PoseArray>();
    pose_array_msg->header.frame_id = msg->header.frame_id;
    pose_array_msg->header.stamp = this->get_clock()->now();

    //i can use two method to check if a cluster it's an apple: it's dimension and if small enough i can check if it's an arc
    for(const auto& cluster: filtered_cluster)
    {
      Point start_point = cluster.front();
      Point end_point = cluster.back(); 

      double dist_x = start_point.x - end_point.x; 
      double dist_y = start_point.y - end_point.y; 
      double cluster_span = std::sqrt(pow(dist_x,2)+pow(dist_y,2));

      if (cluster_span > MAX_APPLE_SPAN) {
        continue; //exit the for not an apple 
      }
      if (!is_arc(cluster, CONVEXITY_TRESHOLD)) {
        //not an arc and thus not an apple
        continue;
      }
      
      //if i'm here it's an apple
      apple_count++; 


      //now i have to find a method to find the center position
      ApplePosition apple_pos = find_apple_position(cluster);
      if (apple_pos.success) 
      {
        geometry_msgs::msg::Pose pose;
        pose.position.x = apple_pos.x;
        pose.position.y = apple_pos.y;
        pose.position.z = 0.0;
        pose.orientation.w = 1.0;
        pose_array_msg->poses.push_back(pose);
      }
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Found %d apples.", apple_count);

    this->visible_apples_ = apple_count;
    this->apple_pub_->publish(std::move(pose_array_msg));

  }

  void find_apples_callback( const std::shared_ptr<FindApples::Request> request,
        std::shared_ptr<FindApples::Response> response)
  {
      int n = request->current_apples;
      int s = request->burrow_size;
      int apples_needed = s - n;
      
      //report
      RCLCPP_INFO(this->get_logger(), "Incoming request: Burrow has %d apples. Needs %d.", n, apples_needed);

      //change success bool if number of apple met 
      if (this->visible_apples_ >= apples_needed)
      {
        response->success = true;
      }
      else
      {
        response->success = false;
      }
       
      //log success and found apple
      CLCPP_INFO(this->get_logger(),
            "Found %d apples. Responding: %s",
            this->visible_apples_,
            response->success ? "true" : "false");
  }
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // Create and spin the server node
    auto turtle_server = std::make_shared<TurtlenodeServer>();
    rclcpp::spin(turtle_server);
    
    rclcpp::shutdown();
    return 0;
}
