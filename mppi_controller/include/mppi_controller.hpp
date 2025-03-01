#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <vector>

// Custom data structures to replace ROS interfaces
struct Point {
  double x;
  double y;
};

struct Pose {
  Point position;
  double orientation; // Quaternion simplified to yaw angle for 2D
};

struct Twist {
  double linear_x;
  double linear_y;
  double angular_z;
};

struct Path {
  std::vector<Pose> poses;
};

struct Odometry {
  Pose pose;
  Twist twist;
  std::chrono::system_clock::time_point stamp;
};

struct MPPIParams {
  // General parameters
  int batch_size;
  int time_steps;
  double model_dt;

  // Cost function parameters
  double obstacle_cost_weight;
  double path_cost_weight;
  double smoothness_cost_weight;
  double goal_cost_weight;

  // MPPI specific parameters
  double temperature;
  double gamma;
  double noise_variance;

  // Trajectory generation parameters
  double vx_max;
  double vx_min;
  double vy_max;
  double vy_min;
  double wz_max;
  double wz_min;
  double vx_rate_limit;
  double vy_rate_limit;
  double wz_rate_limit;
};

struct CostmapInfo {
  std::vector<std::vector<uint8_t>> costmap;
  double resolution;
  Point origin;
  int width;
  int height;
};

class CustomMPPIController {
public:
  CustomMPPIController(const MPPIParams &params);
  ~CustomMPPIController() = default;

  // Initialize the controller with parameters
  void initialize(const MPPIParams &params);

  // Main control loop
  Twist computeVelocityCommands(const Odometry *current_odom, const Path *path,
                                const CostmapInfo *costmap_info);

  // Set new path
  void setPath(const Path *path);

  // Reset controller state
  void reset();

private:
  // Parameters
  MPPIParams params_;

  // State variables
  Path current_path_;
  Odometry current_odom_;
  std::vector<Twist> optimal_control_sequence_;

  // Random number generation
  std::mt19937 gen_;
  std::normal_distribution<double> normal_dist_;

  // Internal methods
  void generateNoisyTrajectories(std::vector<std::vector<Twist>> &trajectories,
                                 const std::vector<Twist> &control_sequence);

  void propagateTrajectories(
      std::vector<std::vector<Pose>> &state_trajectories,
      const std::vector<std::vector<Twist>> &control_trajectories,
      const Pose &start_pose);

  void computeTrajectoryCosts(
      std::vector<double> &trajectory_costs,
      const std::vector<std::vector<Pose>> &state_trajectories,
      const std::vector<std::vector<Twist>> &control_trajectories,
      const Path &reference_path, const CostmapInfo &costmap_info);

  void
  updateControlSequence(std::vector<Twist> &control_sequence,
                        const std::vector<std::vector<Twist>> &noisy_controls,
                        const std::vector<double> &costs);

  double computePathCost(const std::vector<Pose> &trajectory,
                         const Path &reference_path);

  double computeObstacleCost(const std::vector<Pose> &trajectory,
                             const CostmapInfo &costmap_info);

  double computeGoalCost(const std::vector<Pose> &trajectory,
                         const Path &reference_path);

  double computeSmoothnessCost(const std::vector<Twist> &controls);

  Pose predictMotionModel(const Pose &start, const Twist &control, double dt);

  double getYaw(double orientation);

  int getCostmapIndex(const Point &point, const CostmapInfo &costmap_info);
};