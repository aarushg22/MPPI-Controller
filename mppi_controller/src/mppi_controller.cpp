#include "mppi_controller.hpp"
#include <algorithm>
#include <iostream>
#include <limits>

CustomMPPIController::CustomMPPIController(const MPPIParams &params)
    : params_(params), gen_(std::random_device{}()), normal_dist_(0.0, 1.0) {

  // Initialize optimal control sequence
  optimal_control_sequence_.resize(params_.time_steps);
  for (auto &control : optimal_control_sequence_) {
    control.linear_x = 0.0;
    control.linear_y = 0.0;
    control.angular_z = 0.0;
  }
}

void CustomMPPIController::initialize(const MPPIParams &params) {
  params_ = params;

  // Resize control sequence
  optimal_control_sequence_.resize(params_.time_steps);
  for (auto &control : optimal_control_sequence_) {
    control.linear_x = 0.0;
    control.linear_y = 0.0;
    control.angular_z = 0.0;
  }
}

void CustomMPPIController::reset() {
  // Reset the optimal control sequence
  for (auto &control : optimal_control_sequence_) {
    control.linear_x = 0.0;
    control.linear_y = 0.0;
    control.angular_z = 0.0;
  }
}

void CustomMPPIController::setPath(const Path *path) {
  if (path) {
    current_path_ = *path;
  }
}

Twist CustomMPPIController::computeVelocityCommands(
    const Odometry *current_odom, const Path *path,
    const CostmapInfo *costmap_info) {
  if (!current_odom || !path || !costmap_info) {
    // Return zero velocity if any input is missing
    Twist zero_vel;
    zero_vel.linear_x = 0.0;
    zero_vel.linear_y = 0.0;
    zero_vel.angular_z = 0.0;
    return zero_vel;
  }

  // Update current state and path
  current_odom_ = *current_odom;
  current_path_ = *path;

  // Allocate memory for trajectories and costs
  std::vector<std::vector<Twist>> noisy_controls(
      params_.batch_size, std::vector<Twist>(params_.time_steps));

  std::vector<std::vector<Pose>> state_trajectories(
      params_.batch_size, std::vector<Pose>(params_.time_steps + 1));

  std::vector<double> trajectory_costs(params_.batch_size, 0.0);

  // Generate noisy control trajectories
  generateNoisyTrajectories(noisy_controls, optimal_control_sequence_);

  // Propagate trajectories using motion model
  propagateTrajectories(state_trajectories, noisy_controls, current_odom_.pose);

  // Compute costs for each trajectory
  computeTrajectoryCosts(trajectory_costs, state_trajectories, noisy_controls,
                         current_path_, *costmap_info);

  // Update optimal control sequence
  updateControlSequence(optimal_control_sequence_, noisy_controls,
                        trajectory_costs);

//   std::cout << "Optimal Control Sequence: " << std::endl;
//   for (int i = 0; i < params_.time_steps; ++i) {
//     std::cout << "  Time step " << i
//               << ": linear_x = " << optimal_control_sequence_[i].linear_x
//               << ", linear_y = " << optimal_control_sequence_[i].linear_y
//               << ", angular_z = " << optimal_control_sequence_[i].angular_z
//               << std::endl;
//   }
  // Return first control in the sequence
  return optimal_control_sequence_[0];
}

void CustomMPPIController::generateNoisyTrajectories(
    std::vector<std::vector<Twist>> &trajectories,
    const std::vector<Twist> &control_sequence) {
  for (int i = 0; i < params_.batch_size; ++i) {
    for (int j = 0; j < params_.time_steps; ++j) {
      // Base control
      trajectories[i][j] = control_sequence[j];

      // Add noise scaled by variance
      double noise_scale = sqrt(params_.noise_variance);
      trajectories[i][j].linear_x += noise_scale * normal_dist_(gen_);
      trajectories[i][j].linear_y += noise_scale * normal_dist_(gen_);
      trajectories[i][j].angular_z += noise_scale * normal_dist_(gen_);

      // Apply constraints
      trajectories[i][j].linear_x = std::clamp(trajectories[i][j].linear_x,
                                               params_.vx_min, params_.vx_max);
      trajectories[i][j].linear_y = std::clamp(trajectories[i][j].linear_y,
                                               params_.vy_min, params_.vy_max);
      trajectories[i][j].angular_z = std::clamp(trajectories[i][j].angular_z,
                                                params_.wz_min, params_.wz_max);

      // Apply rate limits if not the first timestep
      if (j > 0) {
        double dt = params_.model_dt;
        const auto &prev = trajectories[i][j - 1];

        double max_vx_change = params_.vx_rate_limit * dt;
        double max_vy_change = params_.vy_rate_limit * dt;
        double max_wz_change = params_.wz_rate_limit * dt;

        trajectories[i][j].linear_x = std::clamp(trajectories[i][j].linear_x,
                                                 prev.linear_x - max_vx_change,
                                                 prev.linear_x + max_vx_change);

        trajectories[i][j].linear_y = std::clamp(trajectories[i][j].linear_y,
                                                 prev.linear_y - max_vy_change,
                                                 prev.linear_y + max_vy_change);

        trajectories[i][j].angular_z = std::clamp(
            trajectories[i][j].angular_z, prev.angular_z - max_wz_change,
            prev.angular_z + max_wz_change);
      }
    }
  }
}

void CustomMPPIController::propagateTrajectories(
    std::vector<std::vector<Pose>> &state_trajectories,
    const std::vector<std::vector<Twist>> &control_trajectories,
    const Pose &start_pose) {
  for (int i = 0; i < params_.batch_size; ++i) {
    // Set initial state
    state_trajectories[i][0] = start_pose;

    // Propagate using motion model
    for (int j = 0; j < params_.time_steps; ++j) {
      state_trajectories[i][j + 1] =
          predictMotionModel(state_trajectories[i][j],
                             control_trajectories[i][j], params_.model_dt);
    }
  }
}

Pose CustomMPPIController::predictMotionModel(const Pose &start,
                                              const Twist &control, double dt) {
  Pose next;

  // Simple differential drive model
  double yaw = getYaw(start.orientation);

  // Update position based on velocity
  next.position.x = start.position.x + control.linear_x * cos(yaw) * dt -
                    control.linear_y * sin(yaw) * dt;
  next.position.y = start.position.y + control.linear_x * sin(yaw) * dt +
                    control.linear_y * cos(yaw) * dt;

  // Update orientation
  next.orientation = start.orientation + control.angular_z * dt;

  // Normalize orientation to [-π, π]
  while (next.orientation > M_PI)
    next.orientation -= 2.0 * M_PI;
  while (next.orientation < -M_PI)
    next.orientation += 2.0 * M_PI;

  return next;
}

double CustomMPPIController::getYaw(double orientation) {
  return orientation; // Simplified for this implementation
}

void CustomMPPIController::computeTrajectoryCosts(
    std::vector<double> &trajectory_costs,
    const std::vector<std::vector<Pose>> &state_trajectories,
    const std::vector<std::vector<Twist>> &control_trajectories,
    const Path &reference_path, const CostmapInfo &costmap_info) {
  for (int i = 0; i < params_.batch_size; ++i) {
    // Compute individual cost components
    double path_cost = computePathCost(state_trajectories[i], reference_path);
    double obstacle_cost =
        computeObstacleCost(state_trajectories[i], costmap_info);
    double goal_cost = computeGoalCost(state_trajectories[i], reference_path);
    double smoothness_cost = computeSmoothnessCost(control_trajectories[i]);

    // Combine costs with weights
    trajectory_costs[i] = params_.path_cost_weight * path_cost +
                          params_.obstacle_cost_weight * obstacle_cost +
                          params_.goal_cost_weight * goal_cost +
                          params_.smoothness_cost_weight * smoothness_cost;
  }
}

double
CustomMPPIController::computePathCost(const std::vector<Pose> &trajectory,
                                      const Path &reference_path) {
  if (reference_path.poses.empty()) {
    return 0.0;
  }

  double total_cost = 0.0;

  for (size_t i = 0; i < trajectory.size(); ++i) {
    // Find closest point on reference path
    double min_dist = std::numeric_limits<double>::max();
    for (const auto &ref_pose : reference_path.poses) {
      double dx = trajectory[i].position.x - ref_pose.position.x;
      double dy = trajectory[i].position.y - ref_pose.position.y;
      double dist = dx * dx + dy * dy;
      min_dist = std::min(min_dist, dist);
    }

    // Add squared distance to cost
    total_cost += min_dist;
  }

  return total_cost;
}

double
CustomMPPIController::computeObstacleCost(const std::vector<Pose> &trajectory,
                                          const CostmapInfo &costmap_info) {
  double total_cost = 0.0;

  for (const auto &pose : trajectory) {
    int idx = getCostmapIndex(pose.position, costmap_info);
    if (idx >= 0) {
      int row = idx / costmap_info.width;
      int col = idx % costmap_info.width;

      // Get cost from costmap (assuming higher values mean higher cost)
      if (row >= 0 && row < costmap_info.height && col >= 0 &&
          col < costmap_info.width) {
        uint8_t cost_value = costmap_info.costmap[row][col];
        // Scale cost appropriately and add to total
        total_cost += static_cast<double>(cost_value) / 255.0;
      }
    }
  }

  return total_cost;
}

double
CustomMPPIController::computeGoalCost(const std::vector<Pose> &trajectory,
                                      const Path &reference_path) {
  if (reference_path.poses.empty()) {
    return 0.0;
  }

  // Get final pose in reference path
  const Pose &goal_pose = reference_path.poses.back();

  // Get final pose in trajectory
  const Pose &final_pose = trajectory.back();

  // Compute distance to goal
  double dx = final_pose.position.x - goal_pose.position.x;
  double dy = final_pose.position.y - goal_pose.position.y;
  double position_error = dx * dx + dy * dy;

  // Compute orientation error
  double yaw_final = getYaw(final_pose.orientation);
  double yaw_goal = getYaw(goal_pose.orientation);
  double orientation_error = std::abs(yaw_final - yaw_goal);
  while (orientation_error > M_PI) {
    orientation_error = std::abs(orientation_error - 2.0 * M_PI);
  }

  // Combine position and orientation errors
  return position_error + 0.5 * orientation_error * orientation_error;
}

double CustomMPPIController::computeSmoothnessCost(
    const std::vector<Twist> &controls) {
  double total_cost = 0.0;

  for (size_t i = 1; i < controls.size(); ++i) {
    // Compute control differences
    double dvx = controls[i].linear_x - controls[i - 1].linear_x;
    double dvy = controls[i].linear_y - controls[i - 1].linear_y;
    double dwz = controls[i].angular_z - controls[i - 1].angular_z;

    // Add squared differences to cost
    total_cost += dvx * dvx + dvy * dvy + dwz * dwz;
  }

  return total_cost;
}

void CustomMPPIController::updateControlSequence(
    std::vector<Twist> &control_sequence,
    const std::vector<std::vector<Twist>> &noisy_controls,
    const std::vector<double> &costs) {
  // Find minimum cost for normalization
  double min_cost = *std::min_element(costs.begin(), costs.end());

  // Compute exponentiated weights
  std::vector<double> weights(params_.batch_size);
  double sum_weights = 0.0;

  for (int i = 0; i < params_.batch_size; ++i) {
    weights[i] = std::exp(-(costs[i] - min_cost) / params_.temperature);
    sum_weights += weights[i];
  }

  // Normalize weights
  if (sum_weights > 0.0) {
    for (auto &w : weights) {
      w /= sum_weights;
    }
  } else {
    // If all weights are zero, use uniform weights
    std::fill(weights.begin(), weights.end(), 1.0 / params_.batch_size);
  }

  // Update control sequence using weighted average
  for (int j = 0; j < params_.time_steps; ++j) {
    Twist weighted_control;
    weighted_control.linear_x = 0.0;
    weighted_control.linear_y = 0.0;
    weighted_control.angular_z = 0.0;

    for (int i = 0; i < params_.batch_size; ++i) {
      weighted_control.linear_x += weights[i] * noisy_controls[i][j].linear_x;
      weighted_control.linear_y += weights[i] * noisy_controls[i][j].linear_y;
      weighted_control.angular_z += weights[i] * noisy_controls[i][j].angular_z;
    }

    // Update with exponential smoothing
    control_sequence[j].linear_x =
        params_.gamma * control_sequence[j].linear_x +
        (1.0 - params_.gamma) * weighted_control.linear_x;
    control_sequence[j].linear_y =
        params_.gamma * control_sequence[j].linear_y +
        (1.0 - params_.gamma) * weighted_control.linear_y;
    control_sequence[j].angular_z =
        params_.gamma * control_sequence[j].angular_z +
        (1.0 - params_.gamma) * weighted_control.angular_z;

    // Apply constraints
    control_sequence[j].linear_x = std::clamp(control_sequence[j].linear_x,
                                              params_.vx_min, params_.vx_max);
    control_sequence[j].linear_y = std::clamp(control_sequence[j].linear_y,
                                              params_.vy_min, params_.vy_max);
    control_sequence[j].angular_z = std::clamp(control_sequence[j].angular_z,
                                               params_.wz_min, params_.wz_max);
  }
}

int CustomMPPIController::getCostmapIndex(const Point &point,
                                          const CostmapInfo &costmap_info) {
  // Convert world coordinates to costmap coordinates
  int mx = static_cast<int>((point.x - costmap_info.origin.x) /
                            costmap_info.resolution);
  int my = static_cast<int>((point.y - costmap_info.origin.y) /
                            costmap_info.resolution);

  // Check if point is inside costmap bounds
  if (mx < 0 || mx >= costmap_info.width || my < 0 ||
      my >= costmap_info.height) {
    return -1;
  }

  // Convert to linear index
  return my * costmap_info.width + mx;
}
