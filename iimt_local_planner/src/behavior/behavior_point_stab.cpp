#include "behavior/behavior_point_stab.h"
#include <queue>
#include <vector>
#include <algorithm>

PointStabBehavior::PointStabBehavior(
    std::string &name,
    std::shared_ptr<CommonAlgorithm> &common_algorithm,
    const LocalPlannerConfig &default_config)
    : BehaviorBase(),
      common_algorithm_(common_algorithm),
      default_config_(default_config)
{
  LOG(INFO) << "BehaviorPointStab的各种参数值:";
  ros::NodeHandle nh;
  graceful_controller_ = std::make_shared<graceful_controller::GracefulController>(name, default_config_, common_algorithm_);
  collision_detector_ = std::make_unique<CostmapCollisionDetector>(&common_algorithm_->costmap_2d_, default_config_, 0);
  state_map_fun_[ControlNavState::IDLE] = std::bind(&PointStabBehavior::ExecIdle, this, std::placeholders::_1, std::placeholders::_2);
  state_map_fun_[ControlNavState::GRACEFUL_CONTROL] = std::bind(&PointStabBehavior::ExecGracefulControl, this, std::placeholders::_1, std::placeholders::_2);
  state_map_fun_[ControlNavState::ROTATION_IN_PLACE] = std::bind(&PointStabBehavior::ExecRotationInPlace, this, std::placeholders::_1, std::placeholders::_2);
  state_map_fun_[ControlNavState::MOVE_TO_GOAL] = std::bind(&PointStabBehavior::ExecMoveToGoal, this, std::placeholders::_1, std::placeholders::_2);
  state_map_fun_[ControlNavState::ROTATION_AT_GOAL] = std::bind(&PointStabBehavior::ExecRotationAtGoal, this, std::placeholders::_1, std::placeholders::_2);
  CreateFsmState();
  LOG(INFO) << "                          pointstab_vel_x_max: " << default_config_.pointstab_params.pointstab_vel_x_max;
  LOG(INFO) << "                          pointstab_theta_tolerance: " << default_config_.pointstab_params.pointstab_theta_tolerance;
  LOG(INFO) << "                          pointstab_vel_theta_max: " << default_config_.pointstab_params.pointstab_vel_theta_max;
  LOG(INFO) << "                          pointstab_vel_theta_min: " << default_config_.pointstab_params.pointstab_vel_theta_min;
  LOG(INFO) << "                          pointstab_xy_tolerance: " << default_config_.pointstab_params.pointstab_xy_tolerance;
  LOG(INFO) << "                          pointstab_vel_x_min: " << default_config_.pointstab_params.pointstab_vel_x_min;
  LOG(INFO) << "                          pointstab_max_rot_vel_in_place: " << default_config_.pointstab_params.pointstab_max_rot_vel_in_place;
  LOG(INFO) << "                          pointstab_min_rot_vel_in_place: " << default_config_.pointstab_params.pointstab_min_rot_vel_in_place;
  LOG(INFO) << "                          pointstab_angle_tolerance_in_place: " << default_config_.pointstab_params.pointstab_angle_tolerance_in_place;
  LOG(INFO) << "                          pointstab_sim_period: " << default_config_.pointstab_params.pointstab_sim_period;
  LOG(INFO) << "                          pointstab_step_size: " << default_config_.pointstab_params.pointstab_step_size;
  LOG(INFO) << "                          pointstab_use_graceful_controller: " << default_config_.pointstab_params.pointstab_use_graceful_controller;
  LOG(WARNING) << "PointStabBehavior初始化成功";
}
void PointStabBehavior::ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
  // LOG(INFO) << "PointStabBehavior::ComputeVel...";
  status.status = NavStatus::STATUS_RUNNING;
  auto state = state_machine_.GetState();
  ControlNavEvent event;
  if (state_map_fun_.find(state) != state_map_fun_.end())
  {
    event = state_map_fun_[state](cmd_vel, status);
  }
  state_machine_.Command(event);
}
void PointStabBehavior::SetPlan(const PoseStampedVector &plan)
{
  if (plan.size() != 1)
  {
    LOG(WARNING) << "PointStabBehavior::SetPlan: plan size is not 1, plan size: " << plan.size();
    return;
  }
  goal_pose_ = plan[0]; 
  SetGoalReached(false);
  SetXYToleranceLatch(false);
  meet_obstacle_start_time_ = ros::Time(0); 
  LOG(INFO) << "PointStabBehavior::SetPlan...";
  state_machine_.SetState(ControlNavState::IDLE);
}
/********************************状态机********************************/
void PointStabBehavior::CreateFsmState()
{
  LOG(WARNING) << "PointStabBehavior::CreateFsmState()...";
  // IDLE状态-->GRACEFUL_CONTROL状态
  state_machine_.On(ControlNavState::IDLE, ControlNavEvent::REQUEST_GRACEFUL_CONTROL) = [&]
  {
    LOG(INFO) << "CreateFsmState: Request from IDLE to GRACEFUL_CONTROL";
    state_machine_.SetState(ControlNavState::GRACEFUL_CONTROL);
  };
  // IDLE状态-->ROTATION_IN_PLACE状态
  state_machine_.On(ControlNavState::IDLE, ControlNavEvent::REQUEST_ROTATION_IN_PLACE) = [&]
  {
    LOG(INFO) << "CreateFsmState: Request from IDLE to ROTATION_IN_PLACE";
    state_machine_.SetState(ControlNavState::ROTATION_IN_PLACE);
  };
  // GRACEFUL_CONTROL状态-->ROTATION_IN_PLACE状态
  state_machine_.On(ControlNavState::GRACEFUL_CONTROL, ControlNavEvent::REQUEST_ROTATION_IN_PLACE) = [&]
  {
    LOG(INFO) << "CreateFsmState: Request from GRACEFUL_CONTROL to ROTATION_IN_PLACE";
    state_machine_.SetState(ControlNavState::ROTATION_IN_PLACE);
  };
  // ROTATION_IN_PLACE状态-->MOVE_TO_GOAL状态
  state_machine_.On(ControlNavState::ROTATION_IN_PLACE, ControlNavEvent::REQUEST_MOVE_TO_GOAL) = [&]
  {
    LOG(INFO) << "CreateFsmState: Request from ROTATION_IN_PLACE to MOVE_TO_GOAL";
    state_machine_.SetState(ControlNavState::MOVE_TO_GOAL);
  };
  // MOVE_TO_GOAL状态-->ROTATION_AT_GOAL状态
  state_machine_.On(ControlNavState::MOVE_TO_GOAL, ControlNavEvent::REQUEST_ROTATION_AT_GOAL) = [&]
  {
    LOG(INFO) << "CreateFsmState: Request from MOVE_TO_GOAL to ROTATION_AT_GOAL";
    state_machine_.SetState(ControlNavState::ROTATION_AT_GOAL);
  };
}
/********************************状态********************************/
ControlNavEvent PointStabBehavior::ExecIdle(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
  LOG(INFO) << "PointStabBehavior::ExecIdle ...";
  SetZeroSpeed(cmd_vel);
  LOG(INFO) << "start pose:" << current_pose_.pose.position.x << " " << current_pose_.pose.position.y;
  LOG(INFO) << "goal pose:" << goal_pose_.pose.position.x << " " << goal_pose_.pose.position.y;
  if (default_config_.pointstab_params.pointstab_use_graceful_controller)
  {
    // 从起点到目标点进行线性差值,得到一条全局路径
    std::vector<geometry_msgs::PoseStamped> path;
    double x0 = current_pose_.pose.position.x;
    double y0 = current_pose_.pose.position.y;
    double x1 = goal_pose_.pose.position.x;
    double y1 = goal_pose_.pose.position.y;
    double dx = x1 - x0;
    double dy = y1 - y0;
    double length = std::sqrt(dx * dx + dy * dy);
    if (length < 1e-6)
    {
      path.push_back(goal_pose_);
    }
    int N = (int)std::max(1.0, (double)std::floor(length / default_config_.pointstab_params.pointstab_step_size));
    // 提取 yaw
    double yaw1 = tf2::getYaw(goal_pose_.pose.orientation);
    for (int i = 0; i <= N; ++i)
    {
      double t = static_cast<double>(i) / N;
      geometry_msgs::PoseStamped pose;
      pose.header = current_pose_.header;
      pose.pose.position.x = x0 + t * dx;
      pose.pose.position.y = y0 + t * dy;
      pose.pose.position.z = 0.0;
      // 计算当前点到下一个点的方向
      if (i < N)
      {
        // 当前点指向下一个点的方向
        double next_x = x0 + (t + 1.0 / N) * dx;
        double next_y = y0 + (t + 1.0 / N) * dy;
        double yaw = atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
      }
      else
      {
        // 最后一个点使用目标点的朝向
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw1);
        pose.pose.orientation = tf2::toMsg(q);
      }
      path.push_back(pose);
    }
    // 设置路径
    if (!graceful_controller_->SetPlan(path, filtered_plan_))
    {
      LOG(ERROR) << "SetPlan failed";
      status.status = NavStatus::STATUS_ERROR_IN_POINTSTAB;
      return ControlNavEvent::NONE;
    }
    return ControlNavEvent::REQUEST_GRACEFUL_CONTROL;
  }
  return ControlNavEvent::REQUEST_ROTATION_IN_PLACE;
}
ControlNavEvent PointStabBehavior::ExecGracefulControl(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
  if (!graceful_controller_->ComputeVel(current_pose_, robot_vel_, filtered_plan_, cmd_vel))
  {
    LOG(ERROR) << "PointStabBehavior::ExecGracefulControl, computeVelocityCommands failed!";
    status.status = NavStatus::STATUS_ERROR_IN_POINTSTAB;
    return ControlNavEvent::NONE;
  }
  bool valid_move = collision_detector_->CheckTrajectoryWithTime(
      Eigen::Vector3f(current_pose_.pose.position.x, current_pose_.pose.position.y, tf::getYaw(current_pose_.pose.orientation)),
      Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Vector3f(cmd_vel.linear.x, 0.0, cmd_vel.angular.z), 1.0);
  if (!valid_move)
  {
    SetZeroSpeed(cmd_vel);
    if (meet_obstacle_start_time_.isZero())
    {
      meet_obstacle_start_time_ = ros::Time::now();
    }
    ros::Duration duration = ros::Time::now() - meet_obstacle_start_time_;
    LOG(WARNING) << "ExecGracefulControl, waiting for obstacle, duration:" << duration.toSec(); 
    if (duration.toSec() > default_config_.pointstab_params.pointstab_obstacle_timeout)
    {
      meet_obstacle_start_time_ = ros::Time(0);
      LOG(ERROR) << "ExecGracefulControl, Timed out waiting for obstacle!";
      status.status = NavStatus::STATUS_ERROR_IN_POINTSTAB;
      return ControlNavEvent::NONE;
    }
    return ControlNavEvent::NONE;
  }
  else
  {
    meet_obstacle_start_time_ = ros::Time(0);
    status.status = NavStatus::STATUS_RUNNING;
    // xy方向到了,防止震荡
    if (IsGoalLatched(filtered_plan_, current_pose_, default_config_.pointstab_params.pointstab_xy_tolerance))
    {
      SetZeroSpeed(cmd_vel);
      double angle_diff = angles::shortest_angular_distance(tf2::getYaw(goal_pose_.pose.orientation), tf2::getYaw(current_pose_.pose.orientation));
      if (fabs(angle_diff) < default_config_.pointstab_params.pointstab_theta_tolerance)
      {

        LOG(INFO) << "Reach the goal position and angle!";
        status.status = NavStatus::STATUS_REACHED_GOAL;
        SetGoalReached(true);
        return ControlNavEvent::NONE; 
      }
      LOG(INFO) << "getting close to the target!";
      return ControlNavEvent::REQUEST_ROTATION_IN_PLACE;
    }
    return ControlNavEvent::NONE;
  }
}
ControlNavEvent PointStabBehavior::ExecRotationInPlace(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
  double theta_diff = common_algorithm_->CalculateAngleRAD(goal_pose_.pose.position.x, goal_pose_.pose.position.y, current_pose_.pose.position.x, current_pose_.pose.position.y);
  switch (RotateToGoal(theta_diff, cmd_vel))
  {
  case RotationState::FAILED:
  {
    SetZeroSpeed(cmd_vel);
    ros::Duration duration = ros::Time::now() - meet_obstacle_start_time_;
    LOG(WARNING) << "ExecRotationInPlace, waiting for obstacle, duration:" << duration.toSec();
    if (duration.toSec() > default_config_.pointstab_params.pointstab_obstacle_timeout)
    {
      LOG(ERROR) << "ExecRotationInPlace, Timed out waiting for obstacle!";
      status.status = NavStatus::STATUS_ERROR_IN_POINTSTAB;
      meet_obstacle_start_time_ = ros::Time(0);
      break;
    }
    break;
  }
  case RotationState::FINISHED:
  {
    SetZeroSpeed(cmd_vel);
    meet_obstacle_start_time_ = ros::Time(0);
    LOG(INFO) << "Aligned with the goal direction!";
    status.status = NavStatus::STATUS_RUNNING;
    return ControlNavEvent::REQUEST_MOVE_TO_GOAL;
  }
  case RotationState::ROTATING:
  {
    status.status = NavStatus::STATUS_ROTATING;
    break;
  }
  default:
  {
    SetZeroSpeed(cmd_vel);
    LOG(ERROR) << "Unknown RotationState encountered!";
    status.status = NavStatus::STATUS_ERROR_IN_POINTSTAB;
    break;
  }
  }
  return ControlNavEvent::NONE;
}
ControlNavEvent PointStabBehavior::ExecMoveToGoal(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
  double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
  double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
  double distance = std::hypot(dx, dy);
  double way_theta = atan2(dy, dx);
  double theta_f = angles::shortest_angular_distance(tf::getYaw(current_pose_.pose.orientation), way_theta);
  if (distance < default_config_.pointstab_params.pointstab_xy_tolerance)
  {
    SetZeroSpeed(cmd_vel);
    meet_obstacle_start_time_ = ros::Time(0);
    return ControlNavEvent::REQUEST_ROTATION_AT_GOAL;
  }
  double speed = default_config_.pointstab_params.pointstab_vel_x_max;
  if (fabs(theta_f) <= 0.01)
  {
    cmd_vel.linear.x = speed;
    cmd_vel.angular.z = 0.0;
  }
  else
  {
    double R = distance * 0.5 / sin(theta_f);
    cmd_vel.linear.x = speed;
    cmd_vel.angular.z = speed / R;
    if (fabs(cmd_vel.angular.z) > default_config_.pointstab_params.pointstab_vel_theta_max)
    {
      cmd_vel.angular.z = cmd_vel.angular.z > 0.0 ? default_config_.pointstab_params.pointstab_vel_theta_max : -default_config_.pointstab_params.pointstab_vel_theta_max;
      cmd_vel.linear.x = cmd_vel.angular.z * R;
      cmd_vel.linear.x = std::min(default_config_.pointstab_params.pointstab_vel_x_max, std::max(cmd_vel.linear.x, default_config_.pointstab_params.pointstab_vel_x_min));
    }
  }
  bool valid_move = collision_detector_->CheckTrajectoryWithTime(
      Eigen::Vector3f(current_pose_.pose.position.x, current_pose_.pose.position.y, tf::getYaw(current_pose_.pose.orientation)),
      Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Vector3f(cmd_vel.linear.x, 0.0, cmd_vel.angular.z),
      1.0);
  if (!valid_move)
  {
    SetZeroSpeed(cmd_vel);
    if (meet_obstacle_start_time_.isZero())
    {
      meet_obstacle_start_time_ = ros::Time::now();
    }
    ros::Duration duration = ros::Time::now() - meet_obstacle_start_time_;
    LOG(WARNING) << "ExecMoveToGoal, waiting for obstacle, duration:" << duration.toSec();
    if (duration.toSec() > default_config_.pointstab_params.pointstab_obstacle_timeout)
    {
      LOG(ERROR) << "ExecMoveToGoal, Timed out waiting for obstacle!";
      status.status = NavStatus::STATUS_ERROR_IN_POINTSTAB;
      meet_obstacle_start_time_ = ros::Time(0);
    }
    return ControlNavEvent::NONE;
  }
  meet_obstacle_start_time_ = ros::Time(0);
  status.status = NavStatus::STATUS_RUNNING;
  return ControlNavEvent::NONE;
}
ControlNavEvent PointStabBehavior::ExecRotationAtGoal(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
  switch (RotateToGoal(tf::getYaw(goal_pose_.pose.orientation), cmd_vel))
  {
  case RotationState::FAILED:
  {
    SetZeroSpeed(cmd_vel);
    ros::Duration duration = ros::Time::now() - meet_obstacle_start_time_;
    LOG(WARNING) << "ExecRotationAtGoal, waiting for obstacle, duration:" << duration.toSec();
    if (duration.toSec() > default_config_.pointstab_params.pointstab_obstacle_timeout)
    {
      LOG(ERROR) << "ExecRotationAtGoal, Timed out waiting for obstacle!";
      status.status = NavStatus::STATUS_ERROR_IN_POINTSTAB;
      meet_obstacle_start_time_ = ros::Time(0);
      break;
    }
    break;
  }
  case RotationState::FINISHED:
  {
    SetZeroSpeed(cmd_vel);
    LOG(INFO) << "Reached goal!";
    status.status = NavStatus::STATUS_REACHED_GOAL;
    SetGoalReached(true);
    break;
  }
  case RotationState::ROTATING:
  {
    status.status = NavStatus::STATUS_ROTATING;
    break;
  }
  default:
  {
    SetZeroSpeed(cmd_vel);
    LOG(ERROR) << "ExecRotationAtGoal: Unknown RotationState encountered!";
    status.status = NavStatus::STATUS_ERROR_IN_POINTSTAB;
    break;
  }
  }
  return ControlNavEvent::NONE;
}
RotationState PointStabBehavior::RotateToGoal(double goal_angle, geometry_msgs::Twist &cmd_vel)
{
  double goal_diff = std::fabs(angles::shortest_angular_distance(tf::getYaw(current_pose_.pose.orientation), goal_angle));
  // 达到指定的角度
  if (goal_diff < default_config_.pointstab_params.pointstab_theta_tolerance)
  {
    SetZeroSpeed(cmd_vel);
    LOG(INFO) << "Rotate to the specified angle!";
    reverse_rotation_check_ = false;
    return RotationState::FINISHED;
  }
  // 当角度小于40 度以内
  double temp_max_vel_theta, temp_min_vel_theta;
  if (goal_diff < default_config_.pointstab_params.pointstab_angle_tolerance_in_place)
  {
    temp_max_vel_theta = std::max(default_config_.pointstab_params.pointstab_min_rot_vel_in_place, default_config_.pointstab_params.pointstab_max_rot_vel_in_place * 0.5);
    temp_min_vel_theta = default_config_.pointstab_params.pointstab_min_rot_vel_in_place;
  }
  else
  {
    // use max rot vel from param
    temp_max_vel_theta = default_config_.pointstab_params.pointstab_max_rot_vel_in_place;
    temp_min_vel_theta = default_config_.pointstab_params.pointstab_min_rot_vel_in_place;
  }
  // 计算需要旋转的速度，如果不能旋转了，就返回false
  if (!CheckRotateToGoalPossibilityAndCalcVel(current_pose_, robot_vel_, temp_max_vel_theta, temp_min_vel_theta, goal_angle, cmd_vel))
  {
    if (meet_obstacle_start_time_.isZero())
    {
      meet_obstacle_start_time_ = ros::Time::now();
    }
    SetZeroSpeed(cmd_vel);
    reverse_rotation_check_ = false;
    return RotationState::FAILED;
  }
  return RotationState::ROTATING;
}
bool PointStabBehavior::CheckRotateToGoalPossibilityAndCalcVel(
    const geometry_msgs::PoseStamped &global_pose, const geometry_msgs::PoseStamped &robot_vel,
    const double &max_vel_theta, const double &min_vel_theta, double goal_th, geometry_msgs::Twist &cmd_vel)
{
  // 计算可以旋转的速度
  double ang_diff = angles::shortest_angular_distance(tf::getYaw(global_pose.pose.orientation), goal_th);
  double v_theta_samp = std::min(max_vel_theta, std::max(min_vel_theta, std::fabs(ang_diff)));
  v_theta_samp = ang_diff < 0 ? -v_theta_samp : v_theta_samp;
  // 是否正在进行过反向旋转了
  if (reverse_rotation_check_)
  {
    // is_positive_：记录反向旋转执行的时候，角速度是逆时针吗
    v_theta_samp = is_positive_ ? std::fabs(v_theta_samp) : -std::fabs(v_theta_samp);
  }
  // 检查从robot_vel到v_theta_samp速度，机器人会不会碰撞
  if (IsRotationValid(global_pose, robot_vel, v_theta_samp))
  {
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = v_theta_samp;
    LOG(INFO) << "ang_diff:" << ang_diff << ",v_theta_samp:" << v_theta_samp
              << ",reverse_rotation_check:" << reverse_rotation_check_ << ",is_positive:" << is_positive_
              << ",max_vel_theta:" << max_vel_theta << ",min_vel_theta:" << min_vel_theta
              << ",robot_vx: " << robot_vel.pose.position.x
              << ",vw: " << tf2::getYaw(robot_vel.pose.orientation);
    return true;
  }
  else
  {
    if (reverse_rotation_check_)
    {
      // 如果已经进来过 还出现不能旋转的现象 直接返回false
      LOG(WARNING) << "已经翻转加速度了，还是碰撞，进行结束。";
      SetZeroSpeed(cmd_vel);
      return false;
    }
  }
  // 反向旋转判断 -v_theta_samp
  if (IsRotationValid(global_pose, robot_vel, -v_theta_samp))
  {
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = -v_theta_samp;
    reverse_rotation_check_ = true;
    is_positive_ = cmd_vel.angular.z > 0.0; // 记录反向旋转执行的时候，角速度是逆时针吗
    LOG(INFO) << "ang_diff:" << ang_diff << ",v_theta_samp:" << v_theta_samp
              << ",reverse_rotation_check:" << reverse_rotation_check_ << ",is_positive:" << is_positive_
              << ",max_vel_theta:" << max_vel_theta << ",min_vel_theta:" << min_vel_theta
              << ",robot_vx: " << robot_vel.pose.position.x
              << ",vw: " << tf2::getYaw(robot_vel.pose.orientation);
    return true;
  }
  else
  {
    LOG(WARNING) << "反向旋转也碰撞，直接返回";
    SetZeroSpeed(cmd_vel);
    return false;
  }
}
// 检查从robot_vel到v_theta_samp速度，机器人会不会碰撞
bool PointStabBehavior::IsRotationValid(const geometry_msgs::PoseStamped &global_pose, const geometry_msgs::PoseStamped &robot_vel, double v_theta_samp)
{
  if (!collision_detector_)
  {
    LOG(ERROR) << "PointStabBehavior::IsRotationValid dp nullptr";
    return false;
  }
  bool valid_cmd = collision_detector_->CheckTrajectory(
      Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, tf::getYaw(global_pose.pose.orientation)),
      Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, tf::getYaw(robot_vel.pose.orientation)),
      Eigen::Vector3f(0.0, 0.0, v_theta_samp)); // 第三个变量是目标速度
  return valid_cmd;
}
void PointStabBehavior::ReconfigureConfig()
{
}

