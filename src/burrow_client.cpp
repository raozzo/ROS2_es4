#include "rclcpp/rclcpp.hpp"
#include <chrono>   // For timer
#include <time>
#include <cstdlib>
#include <memory>   // For std::make_shared

// Import the custom service interface
#include "turtle_interfaces/srv/find_apples.hpp"

using namespace std::chrono_literals;

class BurrowClient : public rclcpp::Node
{
  //alias to recall the alias easier
  using FindApples = turtle_interfaces::srv::FindApples;
 
  public:BurrowClient() : Node("burrow_client")
  {
    this->client_ = this->create_client<FindApples>("find_apples");
    while (!client_->wait_for_service(1s)) 
    {
      if (!rclcpp::ok())
      {
              RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
             return;
      }
      RCLCPP_INFO(this->get_logger(), "Wating service to be avaiable");
    }
    RCLCPP_INFO(this->get_logger(), "Service is ready!");

    //seed for random generatore
    srand(time(NULL));

    this->timer_ = this->create_wall_timer(4s, std::bind(&BurrowClient::send_request, this));
  }
  
  private:
  
  void send_request()
  {
    //generate apple in burrow (s is the capacity of the borrow and n is the number of apples already in the borrow)
    //
    int s = rand() % 16 + 5; 
    int n;

    // Generate a probability (0.0 to 1.0)
    double probability = (double)rand() / RAND_MAX;

    if (probability < 0.2) 
    { // 20% chance for n = s
      n = s;
    } 
    else 
    { // 80% chance for n < s
      // Generate 'n' (range 0 to s-1)
      n = rand() % s;
    }

    int apples_needed = s - n;
  
    RCLCPP_INFO(this->get_logger(), "Turtlebot needs to find %d aples!", apples_needed);
      
    //make the request to the server 
    auto request = std::make_shared<FindApples::Request>();
    request->burrow_size = s;
    request->current_apples = n;

    auto response_callback = [this](rclcpp::Client<FindApples>::SharedFuture future) 
    {
      try 
      {
        // Get the result from the future
        auto response = future.get();
        if (response->success) 
        {
         RCLCPP_INFO(this->get_logger(), ">>> Response: Turtlebot FOUND enough apples! :)");
        } 
        else 
        {
          RCLCPP_INFO(this->get_logger(), ">>> Response: Turtlebot did NOT find enough apples. :(");
        }
      } 
      catch (const std::exception &e) 
      {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      }
    };
    
    client_->async_send_request(request, response_callback);
  }

  rclcpp::Client<FindApples>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
}; //burrow client node end

int main (int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto burrow_client_node = std::make_shared<BurrowClient>();
  rclcpp::spin(burrow_client_node);
  rclcpp::shutdown();
  return 0;
}


