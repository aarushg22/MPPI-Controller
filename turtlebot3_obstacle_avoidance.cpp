#include "mppi_controller.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

#define TIME_STEP 64
#define BASE_SPEED 1.5

// Costmap parameters
#define COSTMAP_SIZE 301 // Size of the costmap (301x301 grid for a larger area)
#define COSTMAP_RESOLUTION 0.05 // Each cell represents 5cm x 5cm
#define ROBOT_RADIUS 0.2        // Approximate radius of the robot in meters
#define OBSTACLE_VALUE 255      // Value to mark obstacles
#define FREE_SPACE_VALUE 0      // Value to mark free space
#define UNKNOWN_VALUE 128       // Value to mark unknown areas

// Constants for the TurtleBot3 Burger
#define WHEEL_RADIUS 0.033     // Radius of the wheels in meters
#define WHEEL_SEPARATION 0.160 // Distance between the wheels in meters

// World origin in costmap coordinates
double world_origin_x = 0.0;
double world_origin_y = 0.0;

void twist_to_wheel_speeds(double linear_velocity, double angular_velocity,
                           double &left_speed, double &right_speed) {
  left_speed = (linear_velocity - (angular_velocity * WHEEL_SEPARATION) / 2.0) /
               WHEEL_RADIUS;
  right_speed =
      (linear_velocity + (angular_velocity * WHEEL_SEPARATION) / 2.0) /
      WHEEL_RADIUS;
}

// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * std::sqrt(2.0 * M_PI))) *
         std::exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}


void world_to_map(double world_x, double world_y, int &map_x, int &map_y) {
  map_x = static_cast<int>((world_x - world_origin_x) / COSTMAP_RESOLUTION);
  map_y = static_cast<int>((world_y - world_origin_y) / COSTMAP_RESOLUTION);
}

void map_to_world(int map_x, int map_y, double &world_x, double &world_y) {
  world_x = map_x * COSTMAP_RESOLUTION + world_origin_x;
  world_y = map_y * COSTMAP_RESOLUTION + world_origin_y;
}

void update_costmap(CostmapInfo &costmap, const float *lidar_values,
                    int lidar_width, double lidar_max_range, double robot_x,
                    double robot_y) {
  // Calculate robot position in costmap coordinates
  int robot_map_x, robot_map_y;
  world_to_map(robot_x, robot_y, robot_map_x, robot_map_y);

  // Mark area around the robot as free space (local clearing)
  int clear_radius = static_cast<int>(ROBOT_RADIUS / COSTMAP_RESOLUTION) + 1;
  for (int y = -clear_radius; y <= clear_radius; y++) {
    for (int x = -clear_radius; x <= clear_radius; x++) {
      int map_x = robot_map_x + x;
      int map_y = robot_map_y + y;

      // Check if within bounds
      if (map_x >= 0 && map_x < COSTMAP_SIZE && map_y >= 0 &&
          map_y < COSTMAP_SIZE) {
        // Check if within the circle
        if (x * x + y * y <= clear_radius * clear_radius) {
          costmap.costmap[map_y][map_x] = FREE_SPACE_VALUE;
        }
      }
    }
  }

  // Process each lidar reading
  for (int i = 0; i < lidar_width; i++) {
    if (lidar_values[i] != INFINITY && !std::isnan(lidar_values[i])) {
      // Valid reading, calculate angle and distance
      double angle = (2.0 * M_PI * i) / lidar_width;
      double distance = lidar_values[i];

      // Only process readings within the valid range
      if (distance < lidar_max_range) {
        // Convert polar to cartesian coordinates relative to robot
        double rel_x = distance * std::cos(angle);
        double rel_y = distance * std::sin(angle);

        // Convert to world coordinates
        double obstacle_world_x = robot_x + rel_x;
        double obstacle_world_y = robot_y + rel_y;

        // Convert to costmap coordinates
        int obstacle_map_x, obstacle_map_y;
        world_to_map(obstacle_world_x, obstacle_world_y, obstacle_map_x,
                     obstacle_map_y);

        // Check if within costmap bounds
        if (obstacle_map_x >= 0 && obstacle_map_x < COSTMAP_SIZE &&
            obstacle_map_y >= 0 && obstacle_map_y < COSTMAP_SIZE) {
          costmap.costmap[obstacle_map_y][obstacle_map_x] = OBSTACLE_VALUE;
        }

        // Ray casting to mark free space between robot and obstacle
        double ray_step = COSTMAP_RESOLUTION;
        double ray_length =
            distance - COSTMAP_RESOLUTION; // Stop short of the obstacle

        for (double d = 0.0; d < ray_length; d += ray_step) {
          double ray_x = robot_x + (rel_x * d / distance);
          double ray_y = robot_y + (rel_y * d / distance);

          int ray_map_x, ray_map_y;
          world_to_map(ray_x, ray_y, ray_map_x, ray_map_y);

          if (ray_map_x >= 0 && ray_map_x < COSTMAP_SIZE && ray_map_y >= 0 &&
              ray_map_y < COSTMAP_SIZE) {
            costmap.costmap[ray_map_y][ray_map_x] = FREE_SPACE_VALUE;
          }
        }
      }
    }
  }
}

// Function to display a small section of the costmap around the robot
void display_costmap_section(const CostmapInfo &costmap, int robot_map_x,
                             int robot_map_y, int section_size) {
  int start_x = robot_map_x - section_size / 2;
  int start_y = robot_map_y - section_size / 2;
  int end_x = start_x + section_size;
  int end_y = start_y + section_size;

  // Clamp to costmap boundaries
  start_x = (start_x < 0) ? 0 : start_x;
  start_y = (start_y < 0) ? 0 : start_y;
  end_x = (end_x >= costmap.width) ? costmap.width - 1 : end_x;
  end_y = (end_y >= costmap.height) ? costmap.height - 1 : end_y;

  std::cout << "Costmap section around robot (X = obstacle, . = free, ? = "
               "unknown):\n";

  for (int x = start_x; x < end_x; x++) {
    for (int y = end_y; y > start_y; y--) {
      if (x == robot_map_x && y == robot_map_y) {
        std::cout << "R"; // Robot position
      } else {
        char c = '?'; // Unknown
        if (costmap.costmap[y][x] == OBSTACLE_VALUE) {
          c = 'X';
        } else if (costmap.costmap[y][x] == FREE_SPACE_VALUE) {
          c = '.';
        }
        std::cout << c;
      }
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

bool update_path(const double robot_x, const double robot_y, const double yaw,
                 Path &path) {
  // Check if the robot is close enough to the first pose in the path
  if (!path.poses.empty()) {
    double dx = robot_x - path.poses[0].position.x;
    double dy = robot_y - path.poses[0].position.y;
    double dist_to_first_pose = std::sqrt(dx * dx + dy * dy);
    if (dist_to_first_pose < 0.1) {
      double yaw_diff = std::abs(yaw - path.poses[0].orientation);
      while (yaw_diff > M_PI) {
        yaw_diff -= 2.0 * M_PI;
      }
      if (std::abs(yaw_diff) < 0.1) {
        std::cout << "Robot reached" << path.poses[0].position.x << " "
                  << path.poses[0].position.y << std::endl;
        // Remove the first pose from the path
        path.poses.erase(path.poses.begin());
        return true;
      }
    }
  }
  return false;
}

int main(int argc, char **argv) {
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
  params.vx_max = 0.3;
  params.vx_min = -0.3;
  params.vy_max = 0.3;
  params.vy_min = -0.3;
  params.wz_max = 1.0;
  params.wz_min = -1.0;
  params.vx_rate_limit = 0.05;
  params.vy_rate_limit = 0.05;
  params.wz_rate_limit = 0.1;

  // Initialize controller
  CustomMPPIController controller(params);

  // Create a sample costmap
  CostmapInfo costmap;
  costmap.width = COSTMAP_SIZE;
  costmap.height = COSTMAP_SIZE;
  costmap.resolution = COSTMAP_RESOLUTION;
  costmap.origin.x = 0.0;
  costmap.origin.y = 0.0;
  costmap.costmap.resize(costmap.height,
                         std::vector<uint8_t>(costmap.width, UNKNOWN_VALUE));

  // Initialize Webots
  webots::Supervisor robot;

  // Get robot node for position information
  webots::Node *robot_node = robot.getSelf();

  // Set up lidar
  webots::Lidar *lidar = robot.getLidar("LDS-01");
  lidar->enable(TIME_STEP);
  lidar->enablePointCloud();

  // Set up lidar motors (for visualization)
  webots::Motor *lidar_main_motor = robot.getMotor("LDS-01_main_motor");
  webots::Motor *lidar_secondary_motor =
      robot.getMotor("LDS-01_secondary_motor");
  lidar_main_motor->setPosition(INFINITY);
  lidar_secondary_motor->setPosition(INFINITY);
  lidar_main_motor->setVelocity(30.0);
  lidar_secondary_motor->setVelocity(60.0);

  // Set up wheel motors
  webots::Motor *right_motor = robot.getMotor("right wheel motor");
  webots::Motor *left_motor = robot.getMotor("left wheel motor");
  right_motor->setPosition(INFINITY);
  left_motor->setPosition(INFINITY);
  right_motor->setVelocity(0.0);
  left_motor->setVelocity(0.0);

  // Get lidar properties
  const int lidar_width = lidar->getHorizontalResolution();
  const double lidar_max_range = lidar->getMaxRange();

  // For Default control for the turtlebot lidar example
  // Initialize Braitenberg coefficients
  // std::vector<double> braitenberg_coefficients(lidar_width);
  // for (int i = 0; i < lidar_width; i++) {
  //   braitenberg_coefficients[i] =
  //       6 * gaussian(i, lidar_width / 4, lidar_width / 12);
  // }

  // Get initial robot position to set up the costmap origin
  const double *initial_position = robot_node->getPosition();
  const double *orientation = robot_node->getOrientation();
  double robot_yaw = atan2(orientation[3], orientation[0]);

  // Go forward in a straight line with a pose at every 1 meter
  Path path;
  Pose pose;
  pose.position.x = initial_position[0];
  pose.position.y = initial_position[2]; // Z in Webots is Y in our 2D map
  pose.orientation = robot_yaw;
  path.poses.push_back(pose);
  Pose pose2;
  pose2.position.x = initial_position[0] + 1;
  pose2.position.y = initial_position[2]; // Z in Webots is Y in our 2D map
  pose2.orientation = robot_yaw;
  path.poses.push_back(pose2);
  Pose pose3;
  pose3.position.x = initial_position[0] + 2;
  pose3.position.y = initial_position[2]; // Z in Webots is Y in our 2D map
  pose3.orientation = robot_yaw;
  path.poses.push_back(pose3);

  std::cout << "Path: " << std::endl;
  for (const auto pose : path.poses) {
    std::cout << "pose: x=" << pose.position.x << ", y=" << pose.position.y
              << ", yaw=" << pose.orientation << std::endl;
  }

  // Set the world origin to be centered on the map, with the robot starting
  // near the center
  world_origin_x =
      initial_position[0] - (COSTMAP_SIZE * COSTMAP_RESOLUTION / 2);
  world_origin_y = initial_position[2] - (COSTMAP_SIZE * COSTMAP_RESOLUTION /
                                          2); // Z in Webots is Y in our 2D map

  int step_counter = 0;
  int costmap_display_interval = 50; // Save costmap every 50 steps

  while (robot.step(TIME_STEP) != -1) {
    double left_speed = BASE_SPEED, right_speed = BASE_SPEED;

    // Get current robot position
    const double *position = robot_node->getPosition();
    double robot_x = position[0];
    double robot_y = position[2]; // Z in Webots is Y in our map
    update_path(robot_x, robot_y, robot_yaw, path);

    // Get robot position in costmap coordinates
    int robot_map_x, robot_map_y;
    world_to_map(robot_x, robot_y, robot_map_x, robot_map_y);

    // Check if robot is still within costmap
    if (robot_map_x < 0 || robot_map_x >= COSTMAP_SIZE || robot_map_y < 0 ||
        robot_map_y >= COSTMAP_SIZE) {
      std::cout << "Warning: Robot is outside the costmap boundaries!"
                << std::endl;
      // Could implement costmap shifting here if needed
    }

    // Get lidar values
    const float *lidar_values = lidar->getRangeImage();

    // Update costmap with current lidar readings and robot position
    update_costmap(costmap, lidar_values, lidar_width, lidar_max_range, robot_x,
                   robot_y);

    // Periodically save and display the costmap
    if (step_counter % costmap_display_interval == 0) {
      display_costmap_section(costmap, robot_map_x, robot_map_y, 34);
      std::cout << "Robot position (world): x=" << robot_x << ", y=" << robot_y
                << std::endl;
      std::cout << "Robot position (map): x=" << robot_map_x
                << ", y=" << robot_map_y << std::endl;
    }

    const double *orientation = robot_node->getOrientation();
    double yaw = atan2(orientation[3], orientation[0]);

    Odometry odom;
    odom.pose.position.x = robot_x;
    odom.pose.position.y = robot_y;
    odom.pose.orientation = yaw;
    odom.stamp = std::chrono::system_clock::now();

    // If the path is complete, stop the robot
    if (path.poses.empty()) {
      left_motor->setVelocity(0);
      right_motor->setVelocity(0);
      continue;
    }

    Twist cmd = controller.computeVelocityCommands(&odom, &path, &costmap);
    twist_to_wheel_speeds(cmd.linear_x, cmd.angular_z, left_speed, right_speed);

    // Default control for the turtlebot lidar example
    // // Apply the Braitenberg coefficients on the lidar values
    // for (int i = 0.25 * lidar_width; i < 0.5 * lidar_width; i++) {
    //   const int j = lidar_width - i - 1;
    //   const int k = i - 0.25 * lidar_width;
    //   if (lidar_values[i] != INFINITY && !std::isnan(lidar_values[i]) &&
    //       lidar_values[j] != INFINITY && !std::isnan(lidar_values[j])) {
    //     left_speed += braitenberg_coefficients[k] *
    //                   ((1.0 - lidar_values[i] / lidar_max_range) -
    //                    (1.0 - lidar_values[j] / lidar_max_range));
    //     right_speed += braitenberg_coefficients[k] *
    //                    ((1.0 - lidar_values[j] / lidar_max_range) -
    //                     (1.0 - lidar_values[i] / lidar_max_range));
    //   }
    // }

    // Apply computed velocities
    left_motor->setVelocity(left_speed);
    right_motor->setVelocity(right_speed);

    step_counter++;

  }

  return 0;
}