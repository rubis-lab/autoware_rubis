#include <cinttypes>
#include <cstdlib>
#include <ctime>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

// #include "examples_rclcpp_cbg_executor/ping_node.hpp"
// #include "examples_rclcpp_cbg_executor/utilities.hpp"

#include "rubis_1/rubis_1_node.hpp"

using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;
using namespace std::chrono_literals;

// using examples_rclcpp_cbg_executor::PingNode;
// using examples_rclcpp_cbg_executor::configure_thread;
// using examples_rclcpp_cbg_executor::get_thread_time;
// using examples_rclcpp_cbg_executor::ThreadPriority;

using autoware::rubis_1::Rubis1Node;

/// The main function puts a Ping node in one OS process and runs the
/// experiment. See README.md for an architecture diagram.
int main(int argc, char * argv[])
{
  rclcpp::init(0, nullptr);

  

  std::cout << "hi!" << std::endl;

  std::shared_ptr<Rubis1Node> rubis_node_ptr;

  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  rubis_node_ptr = std::make_shared<Rubis1Node>(node_options);


  // Create one executor within this process.
  rclcpp::executors::SingleThreadedExecutor high_prio_executor;

  high_prio_executor.add_node(rubis_node_ptr);

//   // Create Ping node instance and add it to high-prio executor.
//   auto ping_node = std::make_shared<PingNode>();
//   high_prio_executor.add_node(ping_node);

//   rclcpp::Logger logger = ping_node->get_logger();

  // Create a thread for the executor ...
  auto high_prio_thread = std::thread(
    [&]() {
      high_prio_executor.spin();
    });

//   // ... and configure it accordinly as high prio and pin it to the first CPU.
//   const int CPU_ZERO = 0;
//   bool ret = configure_thread(high_prio_thread, ThreadPriority::HIGH, CPU_ZERO);
//   if (!ret) {
//     RCLCPP_WARN(logger, "Failed to configure high priority thread, are you root?");
//   }

  const std::chrono::seconds EXPERIMENT_DURATION = 30s;
  std::cout << "Running experiment from now on for " << EXPERIMENT_DURATION.count() << "s ..." << std::endl;
  std::this_thread::sleep_for(EXPERIMENT_DURATION);

  // ... and stop the experiment.
  rclcpp::shutdown();
  high_prio_thread.join();

//   ping_node->print_statistics(EXPERIMENT_DURATION);

  return 0;
}