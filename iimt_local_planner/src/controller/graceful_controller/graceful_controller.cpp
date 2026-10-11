#include <base_local_planner/goal_functions.h>
#include "controller/graceful_controller/graceful_controller.h"
using namespace dwa_math_util;

namespace graceful_controller
{
  GracefulController::GracefulController(const std::string &name, const LocalPlannerConfig &default_config, std::shared_ptr<CommonAlgorithm> &common_algorithm)
      : default_config_(default_config),
        common_algorithm_(common_algorithm),
        has_new_path_(false)
  {
    ros::NodeHandle private_nh("~/" + name);
    target_pose_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
    LOG(INFO) << "加载graceful_controller控制器的相关参数:";
    LOG(INFO) << "                    graceful_max_vel_x: " << default_config_.graceful_controller_params.graceful_max_vel_x;
    LOG(INFO) << "                    graceful_min_vel_x: " << default_config_.graceful_controller_params.graceful_min_vel_x;
    LOG(INFO) << "                    graceful_max_vel_theta: " << default_config_.graceful_controller_params.graceful_max_vel_theta;
    LOG(INFO) << "                    graceful_min_in_place_vel_theta: " << default_config_.graceful_controller_params.graceful_min_in_place_vel_theta;
    LOG(INFO) << "                    graceful_xy_goal_tolerance: " << default_config_.graceful_controller_params.graceful_xy_goal_tolerance;
    LOG(INFO) << "                    graceful_k1: " << default_config_.graceful_controller_params.graceful_k1;
    LOG(INFO) << "                    graceful_k2: " << default_config_.graceful_controller_params.graceful_k2;
    LOG(INFO) << "                    graceful_k3: " << default_config_.graceful_controller_params.graceful_k3;
    LOG(INFO) << "                    graceful_beta: " << default_config_.graceful_controller_params.graceful_beta;
    LOG(INFO) << "                    graceful_lambda: " << default_config_.graceful_controller_params.graceful_lambda;
    LOG(INFO) << "                    graceful_min_lookahead: " << default_config_.graceful_controller_params.graceful_min_lookahead;
    LOG(INFO) << "                    graceful_max_lookahead: " << default_config_.graceful_controller_params.graceful_max_lookahead;
    LOG(INFO) << "                    graceful_initial_rotate_tolerance: " << default_config_.graceful_controller_params.graceful_initial_rotate_tolerance;
    LOG(INFO) << "                    graceful_prefer_final_rotation: " << default_config_.graceful_controller_params.graceful_prefer_final_rotation;
    LOG(INFO) << "                    graceful_compute_orientations: " << default_config_.graceful_controller_params.graceful_compute_orientations;
    LOG(INFO) << "                    graceful_use_orientation_filter: " << default_config_.graceful_controller_params.graceful_use_orientation_filter;
    LOG(INFO) << "                    graceful_yaw_filter_tolerance: " << default_config_.graceful_controller_params.graceful_yaw_filter_tolerance;
    LOG(INFO) << "                    graceful_yaw_gap_tolerance: " << default_config_.graceful_controller_params.graceful_yaw_gap_tolerance;
    LOG(INFO) << "                    graceful_latch_xy_goal_tolerance: " << default_config_.graceful_controller_params.graceful_latch_xy_goal_tolerance;
    LOG(INFO) << "                    graceful_scaling_vel_x: " << default_config_.graceful_controller_params.graceful_scaling_vel_x;
    LOG(INFO) << "                    graceful_scaling_step: " << default_config_.graceful_controller_params.graceful_scaling_step;
    LOG(INFO) << "                    graceful_resolution: " << default_config_.graceful_controller_params.graceful_resolution;
    LOG(WARNING) <<"                  Graceful_controller初始化成功";
  }
  bool GracefulController::SetPlan(const std::vector<geometry_msgs::PoseStamped> &plan, std::vector<geometry_msgs::PoseStamped> &filtered_plan)
  {
    std::vector<geometry_msgs::PoseStamped> oriented_plan;
    if (default_config_.graceful_controller_params.graceful_compute_orientations)
    {
      oriented_plan = common_algorithm_->addOrientations(plan);
    }
    else
    {
      oriented_plan = plan;
    }

    if (default_config_.graceful_controller_params.graceful_use_orientation_filter)
    {
      filtered_plan = common_algorithm_->applyOrientationFilter(oriented_plan, default_config_.graceful_controller_params.graceful_yaw_filter_tolerance, default_config_.graceful_controller_params.graceful_yaw_gap_tolerance);
    }
    else
    {
      filtered_plan = oriented_plan;
    }

    if (!filtered_plan.empty())
    {
      has_new_path_ = true;
      goal_tolerance_met_ = false;
      LOG(INFO) << "Recieved a new path with " << filtered_plan.size() << " points";
      return true;
    }
    return false;
  }
  bool GracefulController::ComputeVel(geometry_msgs::PoseStamped &current_pose, geometry_msgs::PoseStamped &current_robot_vel, const PoseStampedVector &transformed_plan, geometry_msgs::Twist &cmd_vel)
  {
    if (transformed_plan.empty())
    {
      LOG(WARNING) << "Received an empty transform plan";
      return false;
    }

    Eigen::Matrix4d transform, inv_transform;
    transform = charge_math_util::PoseStampedToMatrix4d(current_pose); 
    inv_transform = transform.inverse();

    // Get the overall goal
    geometry_msgs::PoseStamped goal_pose;
    goal_pose = transformed_plan.back();

    double robot_vel_x = 0.0, robot_vel_yaw = 0.0;
    bool below_velocity_limits = true; // 表示速度已经满足停车旋转情况了
    robot_vel_x = fabs(current_robot_vel.pose.position.x);
    robot_vel_yaw = fabs(tf2::getYaw(current_robot_vel.pose.orientation));
    if (robot_vel_x > default_config_.graceful_controller_params.graceful_xy_vel_goal_tolerance || robot_vel_yaw > default_config_.graceful_controller_params.graceful_yaw_vel_goal_tolerance)
    {
      below_velocity_limits = false;
    }
    double dist_to_goal = std::hypot(goal_pose.pose.position.x - current_pose.pose.position.x, goal_pose.pose.position.y - current_pose.pose.position.y);
    // 到达目标点后的旋转速度计算
    // if ((dist_to_goal < default_config_.graceful_controller_params.graceful_xy_goal_tolerance && below_velocity_limits) || goal_tolerance_met_)
    // {
    //   goal_tolerance_met_ = default_config_.graceful_controller_params.graceful_latch_xy_goal_tolerance;
    //   common_algorithm_->TransformPoseToBaseLink(inv_transform, transformed_plan.back(), goal_pose);
    //   // 计算机器人旋转指定角度的速度
    //   RotateTowards(tf2::getYaw(goal_pose.pose.orientation), cmd_vel);
    //   LOG(INFO) << "Goal tolerance met. Dist to goal: " << dist_to_goal << " below_velocity_limits: " << below_velocity_limits << " goal_tolerance_met_: " << goal_tolerance_met_;
    //   return true;
    // }

    // Compute distance along path
    std::vector<geometry_msgs::PoseStamped> target_poses; // base_link坐标系下
    std::vector<double> target_distances;
    for (const auto& pose : transformed_plan)
    {
      geometry_msgs::PoseStamped transformed_pose;
      common_algorithm_->TransformPoseToBaseLink(inv_transform, pose, transformed_pose);
      target_poses.push_back(transformed_pose);
    }
    // base_link坐标系下，计算路径上每个点到机器人的距离
    ComputeDistanceAlongPath(target_poses, target_distances);
    // 找到dist_to_target在graceful_min_lookahead和graceful_max_lookahead之间的点，优先选择距离目标点近的点，进行模拟，看看是否可达
    LOG(ERROR) << "-----------transformed_plan.size():" << transformed_plan.size();
    for (int i = transformed_plan.size() - 1; i >= 0; --i)
    {
      // 基础控制律需要一个单一的目标位姿，该目标位姿应当：
      //  * 尽可能远离机器人（以保证平滑性）
      //  * 但不能超过最大前视距离
      //  * 能以无碰撞的方式到达
      geometry_msgs::PoseStamped target_pose = target_poses[i];
      double dist_to_target = target_distances[i];
      if (dist_to_target > default_config_.graceful_controller_params.graceful_max_lookahead) // 不能太远
      {
        continue;
      }
      // 到达目标点了
      if (dist_to_goal < default_config_.graceful_controller_params.graceful_max_lookahead)
      {
        LOG(ERROR) << "11111111111111111111111111";
        if (default_config_.graceful_controller_params.graceful_prefer_final_rotation)
        {
          // Avoid unstability and big sweeping turns at the end of paths by ignoring final heading
          double yaw = std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);
          target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
      }
      else if (dist_to_target < default_config_.graceful_controller_params.graceful_min_lookahead) // 不能太近
      {
        LOG(WARNING) << "dis_to_target less than graceful_min_lookahead";
        break;
      }
      // Iteratively try to find a path, incrementally reducing the velocity
      double sim_velocity = default_config_.graceful_controller_params.graceful_max_vel_x;
      do
      {
        max_abs_velocity_ = sim_velocity;
        if (Simulate(target_pose, cmd_vel))
        {
          LOG(INFO) << "Selected a target pose " << i << " dist_to_target: " << dist_to_target << " target_pose x: " << target_pose.pose.position.x << " y: " << target_pose.pose.position.y << " below_velocity_limits: " << below_velocity_limits << " dist_to_goal: " << dist_to_goal;
          return true;
        }
        sim_velocity -= default_config_.graceful_controller_params.graceful_scaling_step;
        LOG(ERROR) << "sim_velocity:" << sim_velocity;
      } while (sim_velocity >= default_config_.graceful_controller_params.graceful_scaling_vel_x);
    }
    LOG(ERROR) << "No pose in path was reachable";
    return false;
  }

  bool GracefulController::Simulate(const geometry_msgs::PoseStamped &target_pose, geometry_msgs::Twist &cmd_vel)
  {
    std::vector<geometry_msgs::PoseStamped> simulated_path;
    geometry_msgs::PoseStamped start_pose;
    start_pose.header.frame_id = common_algorithm_->costmap_ros_->getBaseFrameID();
    start_pose.pose.position.x = 0.0;
    start_pose.pose.position.y = 0.0;
    start_pose.pose.position.z = 0.0;
    start_pose.pose.orientation.x = 0.0;
    start_pose.pose.orientation.y = 0.0;
    start_pose.pose.orientation.z = 0.0;
    start_pose.pose.orientation.w = 1.0;
    simulated_path.push_back(start_pose);
    // 是否需要起步旋转
    bool sim_initial_rotation = has_new_path_ && default_config_.graceful_controller_params.graceful_initial_rotate_tolerance > 0.0;
    ros::Time start_time = ros::Time::now();
    while (true)
    {
      LOG(ERROR) << "222222222222222222222222222222222";
      if ((ros::Time::now() - start_time).toSec() > 0.05)
      {
        LOG(ERROR) << "timeout for loops!";
        return false;
      }
      
      geometry_msgs::PoseStamped error = target_pose;
      double error_angle = tf2::getYaw(error.pose.orientation);

      // Move origin to our current simulated pose
      double dx = error.pose.position.x - simulated_path.back().pose.position.x;
      double dy = error.pose.position.y - simulated_path.back().pose.position.y;

      double theta = -tf2::getYaw(simulated_path.back().pose.orientation);
      error.pose.position.x = dx * cos(theta) - dy * sin(theta);
      error.pose.position.y = dx * sin(theta) + dy * cos(theta);

      error_angle += theta;
      error.pose.orientation.z = sin(error_angle / 2.0);
      error.pose.orientation.w = cos(error_angle / 2.0);
      double vel_x, vel_th;
      if (sim_initial_rotation)
      {
        geometry_msgs::Twist rotation;
        // 已经旋转到位了
        if (fabs(RotateTowards(error, rotation)) < default_config_.graceful_controller_params.graceful_initial_rotate_tolerance)
        {
          // 这个表示已经旋转到位,并且还是一个点,那么表示已经到达终点了
          if (simulated_path.size() == 1)
          {
            // Current robot pose satisifies initial rotate tolerance
            LOG(WARNING) << "Done rotating towards path";
            has_new_path_ = false;
          }
          sim_initial_rotation = false;
        }
        vel_x = rotation.linear.x;
        vel_th = rotation.angular.z;
      }
      if (!sim_initial_rotation)
      {
        if (!Approach(error.pose.position.x, error.pose.position.y, error_angle, vel_x, vel_th))
        {
          LOG(ERROR) << "Unable to compute approach";
          return false;
        }
      }
      // 模拟的第一次迭代，将我们的命令存储到机器人
      if (simulated_path.size() == 1)
      {
        cmd_vel.linear.x = vel_x;
        cmd_vel.angular.z = vel_th;
      }

      LOG(ERROR) << "-------------------------dis:" << std::hypot(error.pose.position.x, error.pose.position.y);
      LOG(ERROR) << "-------------------------vel_x:" << vel_x << " vel_th:" << vel_th;
      if (std::hypot(error.pose.position.x, error.pose.position.y) < default_config_.graceful_controller_params.graceful_resolution)
      {
        target_pose_pub_.publish(target_pose);
        return true;
      }
      vel_x = 0.0009;
      // Forward simulate command
      geometry_msgs::PoseStamped next_pose = simulated_path.back();
      // Generate next pose
      double dt = (vel_x > 0.0) ? default_config_.graceful_controller_params.graceful_resolution / vel_x : 0.1;
      double yaw = tf2::getYaw(next_pose.pose.orientation);
      next_pose.pose.position.x += dt * vel_x * cos(yaw);
      next_pose.pose.position.y += dt * vel_x * sin(yaw);
      double angular_dt = std::min(dt, 1.0);  // dt 上限 1.0 秒
      yaw += angular_dt * vel_th;
      next_pose.pose.orientation.z = sin(yaw / 2.0);
      next_pose.pose.orientation.w = cos(yaw / 2.0);
      simulated_path.push_back(next_pose);
    }
    LOG(ERROR) << "Did not reach target_pose, but stopped simulating?";
    return false;
  }
  bool GracefulController::Approach(const double &x, const double &y, const double &theta, double &vel_x, double &vel_th, bool backward_motion)
  {
    double r = std::sqrt(x * x + y * y);
    // Orientation base frame relative to r_
    double delta = (backward_motion) ? std::atan2(-y, -x) : std::atan2(-y, x);
    // Determine orientation of goal frame relative to r_
    double theta2 = angles::normalize_angle(theta + delta);
    // Compute the virtual control
    double a = std::atan(-default_config_.graceful_controller_params.graceful_k1 * theta2);
    // Compute curvature (k)
    double k = -1.0 / r * (default_config_.graceful_controller_params.graceful_k2 * (delta - a) + (1 + (default_config_.graceful_controller_params.graceful_k1 / (1 + ((default_config_.graceful_controller_params.graceful_k1 * theta2) * (default_config_.graceful_controller_params.graceful_k1 * theta2))))) * sin(delta));
    double v = max_abs_velocity_ / (1 + default_config_.graceful_controller_params.graceful_beta * std::pow(fabs(k), default_config_.graceful_controller_params.graceful_lambda));
    v = std::min(std::max(v, default_config_.graceful_controller_params.graceful_min_vel_x), max_abs_velocity_);
    if (backward_motion)
    {
      v *= -1;
    }
    double w = k * v;
    double bounded_w = std::min(default_config_.graceful_controller_params.graceful_max_vel_theta, std::max(-default_config_.graceful_controller_params.graceful_max_vel_theta, w));
    // Make sure that if we reduce w, we reduce v so that kurvature is still followed
    if (w != 0.0)
    {
      v *= (bounded_w / w);
    }
    vel_x = v;
    vel_th = bounded_w;
    return true;
  }
  double GracefulController::RotateTowards(const geometry_msgs::PoseStamped &pose, geometry_msgs::Twist &cmd_vel)
  {
    double yaw = 0.0;
    if (std::hypot(pose.pose.position.x, pose.pose.position.y) > 0.5)
    {
      // Goal is far away, point towards goal
      yaw = std::atan2(pose.pose.position.y, pose.pose.position.x);
    }
    else
    {
      // Goal is nearby, align heading
      yaw = tf2::getYaw(pose.pose.orientation);
    }
    // LOG(INFO) << "graceful_controller, Rotating towards goal, error:" << yaw;
    RotateTowards(yaw, cmd_vel);
    return yaw;
  }
  // 计算机器人旋转指定角度的速度
  void GracefulController::RotateTowards(const double &yaw, geometry_msgs::Twist &cmd_vel)
  {
    double max_vel_th = std::max(default_config_.graceful_controller_params.graceful_max_vel_theta, default_config_.graceful_controller_params.graceful_min_in_place_vel_theta);
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = default_config_.graceful_controller_params.graceful_k3 * fabs(yaw);
    cmd_vel.angular.z = dwa_math_util::sign(yaw) * std::min(max_vel_th, std::max(default_config_.graceful_controller_params.graceful_min_in_place_vel_theta, cmd_vel.angular.z));
  }
  // 局部坐标系下，计算路径上每个点到机器人的距离
  void GracefulController::ComputeDistanceAlongPath(const std::vector<geometry_msgs::PoseStamped> &poses, std::vector<double> &distances)
  {
    distances.resize(poses.size());
    for (size_t i = 0; i < poses.size(); ++i)
    {
      distances[i] = std::hypot(poses[i].pose.position.x, poses[i].pose.position.y);
    }
    auto closest = std::min_element(std::begin(distances), std::end(distances));
    // Sum distances between poses, starting with the closest pose
    // Yes, the poses behind the robot will still use euclidean distance from robot, but we don't use those anyways
    for (size_t i = std::distance(std::begin(distances), closest) + 1; i < distances.size(); ++i)
    {
      distances[i] = distances[i - 1] + std::hypot(poses[i].pose.position.x - poses[i - 1].pose.position.x, poses[i].pose.position.y - poses[i - 1].pose.position.y);
    }
  }
} // namespace graceful_controller
