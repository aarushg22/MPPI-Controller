#include "mppi_controller.hpp"
#include <iostream>

int main() {
  // Create MPPI parameters
  MPPIParams params;
  params.batch_size = 1000;
  params.time_steps = 20;
  params.model_dt = 0.1;
  params.obstacle_cost_weight = 1.0;
  params.path_cost_weight = 1.0;
  params.smoothness_cost_weight = 0.1;
  params.goal_cost_weight = 0.5;
  params.temperature = 0.5;
  params.gamma = 0.9;
  params.noise_variance = 0.1;
  params.vx_max = 0.5;
  params.vx_min = -0.5;
  params.vy_max = 0.5;
  params.vy_min = -0.5;
  params.wz_max = 1.0;
  params.wz_min = -1.0;
  params.vx_rate_limit = 0.1;
  params.vy_rate_limit = 0.1;
  params.wz_rate_limit = 0.2;

  // Initialize controller
  CustomMPPIController controller(params);

  // Create a sample path
  Path path;
  for (int i = 0; i < 10; ++i) {
    Pose pose;
    pose.position.x = i * 0.5;
    // pose.position.y = i * 0.2;
    pose.orientation = atan2(0, 0.5); // Direction of path
    path.poses.push_back(pose);
  }

  // Create a sample costmap
  CostmapInfo costmap;
  costmap.width = 100;
  costmap.height = 100;
  costmap.resolution = 0.1;
  costmap.origin.x = -5.0;
  costmap.origin.y = -5.0;
  costmap.costmap.resize(costmap.height,
                         std::vector<uint8_t>(costmap.width, 0));

  // Add some obstacles to the costmap
  for (int i = 30; i < 40; ++i) {
    for (int j = 30; j < 60; ++j) {
      costmap.costmap[i][j] = 255; // High cost for obstacles
    }
  }

  // Create a sample odometry
  Odometry odom;
  odom.pose.position.x = 0.0;
  odom.pose.position.y = 0.0;
  odom.pose.orientation = 0.0;
  odom.stamp = std::chrono::system_clock::now();

  // Compute velocity command
  Twist cmd = controller.computeVelocityCommands(&odom, &path, &costmap);

  std::cout << "Computed velocity command:" << std::endl;
  std::cout << "  linear_x: " << cmd.linear_x << std::endl;
  std::cout << "  linear_y: " << cmd.linear_y << std::endl;
  std::cout << "  angular_z: " << cmd.angular_z << std::endl;

  return 0;
}