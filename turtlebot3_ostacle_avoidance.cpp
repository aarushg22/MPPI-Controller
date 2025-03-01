/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "mppi_controller.hpp"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 64
#define BASE_SPEED 1.5

// Costmap parameters
#define COSTMAP_SIZE 301 // Size of the costmap (201x201 grid for a larger area)
#define COSTMAP_RESOLUTION 0.05 // Each cell represents 10cm x 10cm
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

// Function to convert Twist to wheel speeds
void twist_to_wheel_speeds(double linear_velocity, double angular_velocity,
                           double *left_speed, double *right_speed) {
  *left_speed =
      (linear_velocity - (angular_velocity * WHEEL_SEPARATION) / 2.0) /
      WHEEL_RADIUS;
  *right_speed =
      (linear_velocity + (angular_velocity * WHEEL_SEPARATION) / 2.0) /
      WHEEL_RADIUS;
}

// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) *
         exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

// Function to initialize the costmap with unknown space
void init_costmap(unsigned char **costmap, int width, int height) {
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      costmap[y][x] = UNKNOWN_VALUE;
    }
  }
}

// Convert world coordinates to costmap indices
void world_to_map(double world_x, double world_y, int *map_x, int *map_y) {
  *map_x = (int)((world_x - world_origin_x) / COSTMAP_RESOLUTION);
  *map_y = (int)((world_y - world_origin_y) / COSTMAP_RESOLUTION);
}

// Convert costmap indices to world coordinates
void map_to_world(int map_x, int map_y, double *world_x, double *world_y) {
  *world_x = map_x * COSTMAP_RESOLUTION + world_origin_x;
  *world_y = map_y * COSTMAP_RESOLUTION + world_origin_y;
}

// Function to update the costmap based on lidar readings and robot position
void update_costmap(unsigned char **costmap, const float *lidar_values,
                    int lidar_width, double lidar_max_range, double robot_x,
                    double robot_y) {
  // Calculate robot position in costmap coordinates
  int robot_map_x, robot_map_y;
  world_to_map(robot_x, robot_y, &robot_map_x, &robot_map_y);

  // Mark area around the robot as free space (local clearing)
  int clear_radius = (int)(ROBOT_RADIUS / COSTMAP_RESOLUTION) + 1;
  for (int y = -clear_radius; y <= clear_radius; y++) {
    for (int x = -clear_radius; x <= clear_radius; x++) {
      int map_x = robot_map_x + x;
      int map_y = robot_map_y + y;

      // Check if within bounds
      if (map_x >= 0 && map_x < COSTMAP_SIZE && map_y >= 0 &&
          map_y < COSTMAP_SIZE) {
        // Check if within the circle
        if (x * x + y * y <= clear_radius * clear_radius) {
          costmap[map_y][map_x] = FREE_SPACE_VALUE;
        }
      }
    }
  }

  // Process each lidar reading
  for (int i = 0; i < lidar_width; i++) {
    if (lidar_values[i] != INFINITY && !isnan(lidar_values[i])) {
      // Valid reading, calculate angle and distance
      double angle = (2.0 * M_PI * i) / lidar_width;
      double distance = lidar_values[i];

      // Only process readings within the valid range
      if (distance < lidar_max_range) {
        // Convert polar to cartesian coordinates relative to robot
        double rel_x = distance * cos(angle);
        double rel_y = distance * sin(angle);

        // Convert to world coordinates
        double obstacle_world_x = robot_x + rel_x;
        double obstacle_world_y = robot_y + rel_y;

        // Convert to costmap coordinates
        int obstacle_map_x, obstacle_map_y;
        world_to_map(obstacle_world_x, obstacle_world_y, &obstacle_map_x,
                     &obstacle_map_y);

        // Check if within costmap bounds
        if (obstacle_map_x >= 0 && obstacle_map_x < COSTMAP_SIZE &&
            obstacle_map_y >= 0 && obstacle_map_y < COSTMAP_SIZE) {
          costmap[obstacle_map_y][obstacle_map_x] = OBSTACLE_VALUE;
        }

        // Ray casting to mark free space between robot and obstacle
        double ray_step = COSTMAP_RESOLUTION;
        double ray_length =
            distance - COSTMAP_RESOLUTION; // Stop short of the obstacle

        for (double d = 0.0; d < ray_length; d += ray_step) {
          double ray_x = robot_x + (rel_x * d / distance);
          double ray_y = robot_y + (rel_y * d / distance);

          int ray_map_x, ray_map_y;
          world_to_map(ray_x, ray_y, &ray_map_x, &ray_map_y);

          if (ray_map_x >= 0 && ray_map_x < COSTMAP_SIZE && ray_map_y >= 0 &&
              ray_map_y < COSTMAP_SIZE) {
            costmap[ray_map_y][ray_map_x] = FREE_SPACE_VALUE;
          }
        }
      }
    }
  }
}

// Function to save costmap to a file
void save_costmap(unsigned char **costmap, int width, int height, int step) {
  char filename[64];
  sprintf(filename, "costmap_%d.pgm", step);

  FILE *fp = fopen(filename, "wb");
  if (!fp) {
    printf("Error: Could not create costmap file\n");
    return;
  }

  // Write PGM header
  fprintf(fp, "P5\n%d %d\n255\n", width, height);

  // Write costmap data
  for (int y = 0; y < height; y++) {
    fwrite(costmap[y], 1, width, fp);
  }

  fclose(fp);
  printf("Costmap saved to %s\n", filename);
}

// Function to display a small section of the costmap around the robot
void display_costmap_section(unsigned char **costmap, int width, int height,
                             int robot_map_x, int robot_map_y,
                             int section_size) {
  int start_x = robot_map_x - section_size / 2;
  int start_y = robot_map_y - section_size / 2;
  int end_x = start_x + section_size;
  int end_y = start_y + section_size;

  // Clamp to costmap boundaries
  start_x = (start_x < 0) ? 0 : start_x;
  start_y = (start_y < 0) ? 0 : start_y;
  end_x = (end_x >= width) ? width - 1 : end_x;
  end_y = (end_y >= height) ? height - 1 : end_y;

  printf(
      "Costmap section around robot (X = obstacle, . = free, ? = unknown):\n");
  for (int x = start_x; x < end_x; x++) {
    for (int y = start_y; y < end_y; y++) {

      if (x == robot_map_x && y == robot_map_y) {
        printf("R"); // Robot position
      } else {
        char c = '?';
        if (costmap[y][x] == OBSTACLE_VALUE) {
          c = 'X';
        } else if (costmap[y][x] == FREE_SPACE_VALUE) {
          c = '.';
        }
        printf("%c", c);
      }
    }
    printf("\n");
  }
  printf("\n");
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
  // CostmapInfo costmap;
  // costmap.width = COSTMAP_SIZE;
  // costmap.height = COSTMAP_SIZE;
  // costmap.resolution = 0.05;
  // costmap.origin.x = 0.0;
  // costmap.origin.y = 0.0;
  // costmap.costmap.resize(costmap.height,
  //                        std::vector<uint8_t>(costmap.width, 0));

  wb_robot_init();

  // Enable supervisor to get robot position
  WbNodeRef robot_node = wb_supervisor_node_get_self();

  // get and enable the lidar
  WbDeviceTag lidar = wb_robot_get_device("LDS-01");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);

  // get lidar motor and enable rotation (only for visualization, no effect on
  // the sensor)
  WbDeviceTag lidar_main_motor = wb_robot_get_device("LDS-01_main_motor");
  WbDeviceTag lidar_secondary_motor =
      wb_robot_get_device("LDS-01_secondary_motor");
  wb_motor_set_position(lidar_main_motor, INFINITY);
  wb_motor_set_position(lidar_secondary_motor, INFINITY);
  wb_motor_set_velocity(lidar_main_motor, 30.0);
  wb_motor_set_velocity(lidar_secondary_motor, 60.0);

  // get the motors and enable velocity control
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_motor_set_velocity(left_motor, 0.0);

  const int lidar_width = wb_lidar_get_horizontal_resolution(lidar);
  const double lidar_max_range = wb_lidar_get_max_range(lidar);

  // init braitenberg coefficient
  double *braitenberg_coefficients =
      (double *)malloc(sizeof(double) * lidar_width);
  int i;
  for (i = 0; i < lidar_width; i++)
    braitenberg_coefficients[i] =
        6 * gaussian(i, lidar_width / 4, lidar_width / 12);

  // Get initial robot position to set up the costmap origin
  const double *initial_position = wb_supervisor_node_get_position(robot_node);

  // Set the world origin to be centered on the map, with the robot starting
  // near the center
  world_origin_x =
      initial_position[0] - (COSTMAP_SIZE * COSTMAP_RESOLUTION / 2);
  world_origin_y = initial_position[2] - (COSTMAP_SIZE * COSTMAP_RESOLUTION /
                                          2); // Z in Webots is Y in our 2D map

  // Initialize costmap
  unsigned char **costmap =
      (unsigned char **)malloc(COSTMAP_SIZE * sizeof(unsigned char *));
  for (i = 0; i < COSTMAP_SIZE; i++) {
    costmap[i] = (unsigned char *)malloc(COSTMAP_SIZE * sizeof(unsigned char));
  }
  init_costmap(costmap, COSTMAP_SIZE, COSTMAP_SIZE);

  int step_counter = 0;
  int costmap_save_interval = 50; // Save costmap every 50 steps

  while (wb_robot_step(TIME_STEP) != -1) {
    double left_speed = BASE_SPEED, right_speed = BASE_SPEED;

    // Get current robot position
    const double *position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[2]; // Z in Webots is Y in our 2D map

    // Get robot position in costmap coordinates
    int robot_map_x, robot_map_y;
    world_to_map(robot_x, robot_y, &robot_map_x, &robot_map_y);

    // Check if robot is still within costmap
    if (robot_map_x < 0 || robot_map_x >= COSTMAP_SIZE || robot_map_y < 0 ||
        robot_map_y >= COSTMAP_SIZE) {
      printf("Warning: Robot is outside the costmap boundaries!\n");
      // Could implement costmap shifting here if needed
    }

    // get lidar values
    const float *lidar_values = wb_lidar_get_range_image(lidar);

    // update costmap with current lidar readings and robot position
    update_costmap(costmap, lidar_values, lidar_width, lidar_max_range, robot_x,
                   robot_y);

    // Periodically save and display the costmap
    if (step_counter % costmap_save_interval == 0) {
      save_costmap(costmap, COSTMAP_SIZE, COSTMAP_SIZE, step_counter);
      display_costmap_section(costmap, COSTMAP_SIZE, COSTMAP_SIZE, robot_map_x,
                              robot_map_y, 21);
      printf("Robot position (world): x=%.2f, y=%.2f\n", robot_x, robot_y);
      printf("Robot position (map): x=%d, y=%d\n", robot_map_x, robot_map_y);
    }

    // apply the braitenberg coefficients on the resulted values of the lidar
    for (i = 0.25 * lidar_width; i < 0.5 * lidar_width; i++) {
      const int j = lidar_width - i - 1;
      const int k = i - 0.25 * lidar_width;
      if (lidar_values[i] != INFINITY && !isnan(lidar_values[i]) &&
          lidar_values[j] != INFINITY && !isnan(lidar_values[j])) {
        left_speed += braitenberg_coefficients[k] *
                      ((1.0 - lidar_values[i] / lidar_max_range) -
                       (1.0 - lidar_values[j] / lidar_max_range));
        right_speed += braitenberg_coefficients[k] *
                       ((1.0 - lidar_values[j] / lidar_max_range) -
                        (1.0 - lidar_values[i] / lidar_max_range));
      }
    }

    // apply computed velocities
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);

    step_counter++;
  };

  // Free allocated memory
  free(braitenberg_coefficients);
  for (i = 0; i < COSTMAP_SIZE; i++) {
    free(costmap[i]);
  }
  free(costmap);

  wb_robot_cleanup();
  return 0;
}