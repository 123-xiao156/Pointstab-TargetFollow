#include "behavior/behavior_charging.h"

ChargingBehavior::ChargingBehavior(std::shared_ptr<CommonAlgorithm> &common_algorithm, const LocalPlannerConfig &default_config)
    : BehaviorBase(),
      common_algorithm_(common_algorithm),
      default_config_(default_config)
{
  ros::NodeHandle nh;
  charge_controller_ = std::make_shared<charge_controller::ChargeController>(nh, default_config_);
  state_map_fun_[ChargeBehaviorState::IDLE] = std::bind(&ChargingBehavior::ExecIDLE, this, std::placeholders::_1, std::placeholders::_2);
  state_map_fun_[ChargeBehaviorState::START_CHARGING] = std::bind(&ChargingBehavior::ExecStartCharge, this, std::placeholders::_1, std::placeholders::_2);
  state_map_fun_[ChargeBehaviorState::END_CHARGING] = std::bind(&ChargingBehavior::ExecEndCharge, this, std::placeholders::_1, std::placeholders::_2);
  CreateFsmState();
  LOG(INFO) << "BehaviorCharging的各种参数值:";
  LOG(INFO) << "                          charge_tags_distance: " << default_config_.charge_params.charge_tags_distance;
  LOG(INFO) << "                          charge_tags_angle_threshold: " << default_config_.charge_params.charge_tags_angle_threshold;
  LOG(INFO) << "                          charge_delay_time: " << default_config_.charge_params.charge_delay_time;
  LOG(INFO) << "                          charge_delay_check_pole_time: " << default_config_.charge_params.charge_delay_check_pole_time;
  LOG(INFO) << "                          charge_waitpoint_leave_pile_dis: " << default_config_.charge_params.charge_waitpoint_leave_pile_dis;
  LOG(INFO) << "                          charge_min_obstacle_points_num: " << default_config_.charge_params.charge_min_obstacle_points_num;
  LOG(INFO) << "                          charge_judge_back_depth: " << default_config_.charge_params.charge_judge_back_depth;
  LOG(INFO) << "                          charge_aim_pile_avoid_obstacle: " << default_config_.charge_params.charge_aim_pile_avoid_obstacle;
  LOG(INFO) << "                          charge_leave_pile_avoid_obstacle: " << default_config_.charge_params.charge_leave_pile_avoid_obstacle;
  LOG(INFO) << "                          charge_final_goal_pid_dis_param: " << default_config_.charge_params.charge_final_goal_pid_dis_param;
  LOG(INFO) << "                          charge_enable_check_reflector: " << default_config_.charge_params.charge_enable_check_reflector;
  LOG(INFO) << "                          charge_leave_pile_avoid_dis: " << default_config_.charge_params.charge_leave_pile_avoid_dis;
  LOG(INFO) << "                          charge_aim_pile_avoid_dis: " << default_config_.charge_params.charge_aim_pile_avoid_dis;
  LOG(INFO) << "                          charge_leave_pile_dist: " << default_config_.charge_params.charge_leave_pile_dist;
  LOG(INFO) << "                          charge_judge_obsacle_depth: " << default_config_.charge_params.charge_judge_obsacle_depth;
  LOG(INFO) << "                          charge_robot_width: " << default_config_.charge_params.charge_robot_width;
  LOG(INFO) << "                          charge_max_v: " << default_config_.charge_params.charge_max_v;
  LOG(INFO) << "                          charge_min_v: " << default_config_.charge_params.charge_min_v;
  LOG(INFO) << "                          charge_max_w: " << default_config_.charge_params.charge_max_w;
  LOG(INFO) << "                          charge_min_w: " << default_config_.charge_params.charge_min_w;
  LOG(INFO) << "                          charge_goal_threshold: " << default_config_.charge_params.charge_goal_threshold;
  LOG(INFO) << "                          charge_visualize: " << default_config_.charge_params.charge_visualize;
  LOG(INFO) << "                          charge_max_angle_with_v: " << default_config_.charge_params.charge_max_angle_with_v;
  LOG(INFO) << "                          charge_pid_param: " << default_config_.charge_params.charge_pid_param;
  LOG(INFO) << "                          charge_control_mode: " << default_config_.charge_params.charge_control_mode;
  LOG(INFO) << "                          charge_leavepile_mode: " << default_config_.charge_params.charge_leavepile_mode;
  LOG(INFO) << "                          charge_pid_angle_param: " << default_config_.charge_params.charge_pid_angle_param;
  LOG(INFO) << "                          charge_pid_dis_param: " << default_config_.charge_params.charge_pid_dis_param;
  LOG(INFO) << "                          charge_finsh_back_distance: " << default_config_.charge_params.charge_finsh_back_distance;
  LOG(INFO) << "                          charge_tag_arrived_threshold: " << default_config_.charge_params.charge_tag_arrived_threshold;
  LOG(INFO) << "                          charge_apriltag_pose_in_baselink_x: " << default_config_.charge_params.charge_apriltag_pose_in_baselink_x;
  LOG(INFO) << "                          charge_apriltag_pose_in_baselink_dis: " << default_config_.charge_params.charge_apriltag_pose_in_baselink_dis;
  LOG(INFO) << "                          charge_leavepile_map_in_baselink_dis: " << default_config_.charge_params.charge_leavepile_map_in_baselink_dis;
  LOG(INFO) << "                          charge_pole_direction: " << default_config_.charge_params.charge_pole_direction;
  LOG(INFO) << "                          charge_rot_flag: " << default_config_.charge_params.charge_rot_flag;
  LOG(INFO) << "                          charge_retreat_mode: " << default_config_.charge_params.charge_retreat_mode;
  LOG(INFO) << "                          charge_final_goal_pid_param: " << default_config_.charge_params.charge_final_goal_pid_param;
  LOG(INFO) << "                          car_type: " << default_config_.charge_params.charge_car_type;
  LOG(INFO) << "                          charge_temp_goal: " << default_config_.charge_params.charge_temp_goal[0] << " " << default_config_.charge_params.charge_temp_goal[1] << " " << default_config_.charge_params.charge_temp_goal[2];
  LOG(INFO) << "                          charge_final_goal: " << default_config_.charge_params.charge_final_goal[0] << " " << default_config_.charge_params.charge_final_goal[1] << " " << default_config_.charge_params.charge_final_goal[2];
  LOG(INFO) << "                          charge_cam_in_baselink_pose: " 
            << default_config_.charge_params.charge_cam_in_baselink_pose[0] << " "<< default_config_.charge_params.charge_cam_in_baselink_pose[1] << " "<< default_config_.charge_params.charge_cam_in_baselink_pose[2]
            << default_config_.charge_params.charge_cam_in_baselink_pose[3] << " " << default_config_.charge_params.charge_cam_in_baselink_pose[4] << " " << default_config_.charge_params.charge_cam_in_baselink_pose[5];
  LOG(INFO) << "                          charge_lidar_in_baselink_pose: "
            << default_config_.charge_params.charge_lidar_in_baselink_pose[0] << " " << default_config_.charge_params.charge_lidar_in_baselink_pose[1] << " " << default_config_.charge_params.charge_lidar_in_baselink_pose[2]
            << default_config_.charge_params.charge_lidar_in_baselink_pose[3] << " " << default_config_.charge_params.charge_lidar_in_baselink_pose[4] << " " << default_config_.charge_params.charge_lidar_in_baselink_pose[5];
  LOG(INFO) << "                          charge_map_in_tag_pose: " << default_config_.charge_params.charge_map_in_tag_pose[0] << " " << default_config_.charge_params.charge_map_in_tag_pose[1] << " " << default_config_.charge_params.charge_map_in_tag_pose[2];
  LOG(INFO) << "                          charge_map_in_reflector_pose: " << default_config_.charge_params.charge_map_in_reflector_pose[0] << " " << default_config_.charge_params.charge_map_in_reflector_pose[1] << " " << default_config_.charge_params.charge_map_in_reflector_pose[2];
  LOG(INFO) << "                          charge_map_in_door_reflector_pose: " << default_config_.charge_params.charge_map_in_door_reflector_pose[0] << " " << default_config_.charge_params.charge_map_in_door_reflector_pose[1] << " " << default_config_.charge_params.charge_map_in_door_reflector_pose[2];
  LOG(WARNING) << "                                                            ChargingBehavior初始化成功";
}
void ChargingBehavior::ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
  status.status = NavStatus::STATUS_RUNNING;
  auto state = state_machine_.GetState();
  ChargeBehaviorEvent event;
  if (state_map_fun_.find(state) != state_map_fun_.end())
  {
    event = state_map_fun_[state](cmd_vel, status);
  }
  state_machine_.Command(event);
}
void ChargingBehavior::SetPlan(TaskType task_type)
{
  LOG(INFO) << "ChargingBehavior::SetPlan...";
  charge_controller_->charge_flag_.store(-1);
  current_task_type_ = task_type;
  state_machine_.SetState(ChargeBehaviorState::IDLE);
  count_ = 0;
  SetGoalReached(false);
}
/********************************状态机********************************/
void ChargingBehavior::CreateFsmState()
{
  // IDLE状态-->START_CHARGING状态
  state_machine_.On(ChargeBehaviorState::IDLE, ChargeBehaviorEvent::REQUEST_START_CHARGING) = [&]
  {
    LOG(INFO) << "CreateFsmState: Request from IDLE->START_CHARGING";
    state_machine_.SetState(ChargeBehaviorState::START_CHARGING);
  };
  // IDLE状态-->END_CHARGING状态
  state_machine_.On(ChargeBehaviorState::IDLE, ChargeBehaviorEvent::REQUEST_END_CHARGING) = [&]
  {
    LOG(INFO) << "CreateFsmState: Request from IDLE->END_CHARGING";
    state_machine_.SetState(ChargeBehaviorState::END_CHARGING);
  };
  // START_CHARGING状态-->IDLE状态
  state_machine_.On(ChargeBehaviorState::START_CHARGING, ChargeBehaviorEvent::REQUEST_IDLE) = [&]
  {
    LOG(INFO) << "CreateFsmState: Request from START_CHARGING->IDLE";
    state_machine_.SetState(ChargeBehaviorState::IDLE);
  };
  // END_CHARGING状态-->IDLE状态
  state_machine_.On(ChargeBehaviorState::END_CHARGING, ChargeBehaviorEvent::REQUEST_IDLE) = [&]
  {
    LOG(INFO) << "CreateFsmState: Request from END_CHARGING->IDLE";
    state_machine_.SetState(ChargeBehaviorState::IDLE);
  };
}
/********************************状态********************************/
ChargeBehaviorEvent ChargingBehavior::ExecIDLE(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
  LOG(INFO) << "ChargingBehavior::ExecIDLE ...";
  SetZeroSpeed(cmd_vel);
  if (current_task_type_ == TaskType::AIM_CHARGER)
  {
    if (charge_controller_->charge_flag_.load() == -1)
    {
      count_++;
      LOG(WARNING) << "charge result publisher not found: " << count_;
      if (count_ > default_config_.charge_params.charge_no_publisher_count)
      {
        LOG(WARNING) << "charge result publisher not found more than 10 times count_:" << count_ << ", " << default_config_.charge_params.charge_no_publisher_count;
        count_ = 0;
        status.status = NavStatus::STATUS_ERROR_CHARGER_FAILED;
      }
      return ChargeBehaviorEvent::NONE;
    }
    count_ = 0;
    charge_controller_->ResetAimPileParams();
    LOG(INFO) << "当前任务类型为:AIM_CHARGER!";
    return ChargeBehaviorEvent::REQUEST_START_CHARGING;
  }
  else if (current_task_type_ == TaskType::DETACH_CHARGER)
  {
    charge_controller_->ResetLeavePileParams();
    LOG(INFO) << "当前任务类型为:DETACH_CHARGER!";
    return ChargeBehaviorEvent::REQUEST_END_CHARGING;
  }
  LOG(ERROR) << "当前任务类型错误!";
  return ChargeBehaviorEvent::NONE;
}
ChargeBehaviorEvent ChargingBehavior::ExecStartCharge(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
  if (!charge_controller_)
  {
    LOG(ERROR) << "charge_controller_ is null!";
    return ChargeBehaviorEvent::REQUEST_IDLE;
  }
  charge_controller::AimState aimpile_step = charge_controller_->RunAimPile(cmd_vel);
  status.failed_id = charge_controller_->failed_id_;
  if (aimpile_step == charge_controller::AIM_FINISH)
  {
    SetZeroSpeed(cmd_vel);
    LOG(INFO) << "AIM_FINISH!";
    status.status = NavStatus::STATUS_REACHED_GOAL;
    SetGoalReached(true);
    return ChargeBehaviorEvent::REQUEST_IDLE;
  }
  else if (aimpile_step == charge_controller::AIM_ERROR)
  {
    // 对桩失败
    SetZeroSpeed(cmd_vel);
    LOG(ERROR) << "AIM_ERROR! 电极不带电或未与电极接触!";
    status.status = NavStatus::STATUS_ERROR_CHARGER_FAILED;
    return ChargeBehaviorEvent::REQUEST_IDLE;
  }
  return ChargeBehaviorEvent::NONE;
}
ChargeBehaviorEvent ChargingBehavior::ExecEndCharge(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
  charge_controller::LeaveState leave_pile_step = charge_controller_->RunLeavePile(cmd_vel);
  status.leavepile_dis = charge_controller_->leavepile_map_to_baselink_dis_;
  if (leave_pile_step == charge_controller::LEAVE_FINISH)
  {
    LOG(INFO) << "离桩成功!";
    SetZeroSpeed(cmd_vel);
    SetGoalReached(true);
    status.status = NavStatus::STATUS_REACHED_GOAL;
    return ChargeBehaviorEvent::REQUEST_IDLE;
  }
  else if (leave_pile_step == charge_controller::LEAVE_ERROR)
  {
    LOG(ERROR) << "离桩失败!";
    SetZeroSpeed(cmd_vel);
    status.status = NavStatus::STATUS_ERROR_END_CHARGER_FAILED;
    return ChargeBehaviorEvent::REQUEST_IDLE;
  }
  return ChargeBehaviorEvent::NONE;
}
void ChargingBehavior::ReconfigureConfig()
{
  LOG(INFO) << "充电没有动态参数更改";
}
