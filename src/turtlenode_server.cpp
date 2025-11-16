#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>
#include <cmath>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "turtle_interfaces/srv/find_apples.hpp"

class TurtlebotServer : public rclcpp::Node
{
  public:
  using FindApples = turtle_interfaces::srv::FindApples;
    
  //constructor
  TurtlenodeServer():Node("turtlenode_server")
  {
    //TODO why use the placeholder 1 and 2
    //
    this->srv_ = this->create_service<FindApples>(
        "find_apples",
        std::bind(&TurtlebotServer::find_apples_callback, this, _1, _2));

    this->scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&TurtlebotServer::scan_callback, this, _1));

    this->apple_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
         "/apples", 10);

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
    const double CLUSTER_TRESHOLD = 0.15; 
    
    //TODO tune this prams
    const int MIN_CLUSTER_POINTS = 3;
    const int MAX_CLUSTER_POINTS = 15;

    //vector of  filtered cluster (only those cluster that have enough number of points and not too much)
    std::vector<std::vector<Point>> filtered_cluster;

    //hold points of current cluster
    std::vector<Point> current_cluster;

    //center point 
    Point last_point = {0,0}; 
    

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
    auto turtle_server = std::make_shared<TurtlebotServer>();
    rclcpp::spin(turtle_server);
    
    rclcpp::shutdown();
    return 0;
}
