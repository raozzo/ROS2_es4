#include "rclcpp/rclcpp.hpp"
#include <chrono>   // For timer
#include <random>   // For C++ random number generation
#include <memory>   // For std::make_shared

// Import the custom service interface
#include "turtle_interfaces/srv/find_apples.hpp"

using namespace std::chrono_literals;

class BurrowClient : public rclcpp::Node
{
  //alias to recall the alias easier
  using FindApples = turtle_interfaces::srv::FindApples;
 
  public:


  private:

};

int int main (int argc, char *argv[]) {
  
  return 0;
}


