#include "iimt_local_planner/iimt_planner_ros.h"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iimt_local_planner::IIMTPlannerROS, nav_core::BaseLocalPlanner)

namespace iimt_local_planner
{
  IIMTPlannerROS::IIMTPlannerROS() : initialized_(false)
  {
    // LOG(INFO) << "IIMTPlannerROS 构造函数成功！！！";
  }

  void IIMTPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (isInitialized())
      return;
    if(nullptr == tf || nullptr == costmap_ros){
      LOG(ERROR) << "tf或者costmap_ros为空";
      return;
    }
    name_ = name;
    ros::NodeHandle private_nh("~/" + name);
    boost::mutex::scoped_lock lock(configuration_mutex_);
    all_config_.loadRosParamFromNodeHandle(private_nh);
    dynamic_speed_sub_ = private_nh.subscribe("/set_max_speed", 1, &IIMTPlannerROS::MaxSpeedCallBack, this);
    dynamic_config_change_ = private_nh.subscribe("/dynamic_config_change", 1, &IIMTPlannerROS::DynamicConfigChangeCallBack, this);
    common_algorithm_ = std::make_shared<CommonAlgorithm>(private_nh, tf, costmap_ros, all_config_.odom_topic);
    common_algorithm_->SetAlgorithmParam(all_config_.common_params.local_target_move_dist);

    // 行为
    idle_behavior_executer_ = std::make_shared<BehaviorIdle>(common_algorithm_);
    rotate_behavior_executer_ = std::make_shared<RotateBehavior>(common_algorithm_, all_config_);
    recovery_behavior_executer_ = std::make_shared<RecoveryBehaviors>(name, common_algorithm_, all_config_);
    free_decision_executer_ = std::make_shared<BehaviorFreeNav>(name, rotate_behavior_executer_, recovery_behavior_executer_, common_algorithm_, all_config_);
    fix_follow_decision_executer_ = std::make_shared<BehaviorFixNav>(name, rotate_behavior_executer_, common_algorithm_, all_config_);
    reloc_behavior_executer_ = std::make_shared<RelocBehaviorExecuter>(common_algorithm_, all_config_);
    if (all_config_.charge_params.is_using_charge_behavior){
      charge_behavior_executer_ = std::make_shared<ChargingBehavior>(common_algorithm_, all_config_);
    }else{
      LOG(WARNING) << "不创建充电行为";
      charge_behavior_executer_ = nullptr;
    }
    point_stab_executer_ = std::make_shared<PointStabBehavior>(name, common_algorithm_, all_config_);
    target_follow_executer_ = std::make_shared<BehaviorTargetFollow>(name, all_config_, common_algorithm_); // 20260804

    behavior_manager_.AddBehavior(LocalBehaviorState::IDLE, static_cast<BehaviorBase *>(idle_behavior_executer_.get()));
    behavior_manager_.AddBehavior(LocalBehaviorState::FREE_PATH, static_cast<BehaviorBase *>(free_decision_executer_.get()));
    behavior_manager_.AddBehavior(LocalBehaviorState::FIX_PATH, static_cast<BehaviorBase *>(fix_follow_decision_executer_.get()));
    behavior_manager_.AddBehavior(LocalBehaviorState::ROTATE_STATE, static_cast<BehaviorBase *>(rotate_behavior_executer_.get()));
    behavior_manager_.AddBehavior(LocalBehaviorState::RECOVERY_STATE, static_cast<BehaviorBase *>(recovery_behavior_executer_.get()));
    behavior_manager_.AddBehavior(LocalBehaviorState::LOCALIZATION, static_cast<BehaviorBase *>(reloc_behavior_executer_.get()));
    if (all_config_.charge_params.is_using_charge_behavior){
      behavior_manager_.AddBehavior(LocalBehaviorState::CHARGING_STATE , static_cast<BehaviorBase *>(charge_behavior_executer_.get()));
    }
    behavior_manager_.AddBehavior(LocalBehaviorState::POINT_STAB , static_cast<BehaviorBase *>(point_stab_executer_.get()));
    behavior_manager_.AddBehavior(LocalBehaviorState::TARGET_FOLLOW , static_cast<BehaviorBase *>(target_follow_executer_.get())); // 20260804
	
    // 20240930  先注释 start
    // dsrv_ = new dynamic_reconfigure::Server<IIMTPlannerConfig>(private_nh);
    // dynamic_reconfigure::Server<IIMTPlannerConfig>::CallbackType cb = boost::bind(&IIMTPlannerROS::reconfigureCB, this, _1, _2);
    // dsrv_->setCallback(cb);
    // 20240930  先注释 end
    state_manager_.SetState(LocalBehaviorState::IDLE);
    initialized_ = true;
    LOG(WARNING) << "       局部规划器初始化成功";
  }

  bool IIMTPlannerROS::setPlan(const PoseStampedVector &orig_global_plan, TaskType path_type)
  {
    if (!isInitialized())
    {
      LOG(ERROR) << "This planner has not been initialized, please call initialize,before using this planner";
      return false;
    }
    boost::mutex::scoped_lock lock(configuration_mutex_);
    if (path_type == TaskType::IDLE)
    {
      // New Task In.
      LOG(INFO) << "Get IDLE task, reste state to idle";
      state_manager_.SetState(LocalBehaviorState::IDLE);
      idle_behavior_executer_->SetPlan();
      return true;
    }
    // 重定位任务
    if (path_type == TaskType::RELOCATION) {
      LOG(INFO) << "rotate to relocalization";
      LOG(INFO) << "设置局部地图为默认参数";
      common_algorithm_->SetOriginalCostmapParam();
      state_manager_.SetState(LocalBehaviorState::LOCALIZATION);
      reloc_behavior_executer_->SetPlan();
      return true;
    }

    if (all_config_.charge_params.is_using_charge_behavior && path_type == TaskType::AIM_CHARGER)
    {
      LOG(INFO) << "Get AIM_CHARGE task,start charging";
      state_manager_.SetState(LocalBehaviorState::CHARGING_STATE);
      LOG(INFO) << "设置LocalBehaviorState为CHARGING_STATE!";
      charge_behavior_executer_->SetPlan(TaskType::AIM_CHARGER);
      return true;
    }else if (path_type == TaskType::AIM_CHARGER){
      LOG(ERROR) << "没有激活充电行为";
      return false;
    }

    if (all_config_.charge_params.is_using_charge_behavior && path_type == TaskType::DETACH_CHARGER)
    {
      LOG(INFO) << "Get DETACH_CHARGE,end charging";
      state_manager_.SetState(LocalBehaviorState::CHARGING_STATE);
      LOG(INFO) << "设置LocalBehaviorState为CHARGING_STATE!";
      charge_behavior_executer_->SetPlan(TaskType::DETACH_CHARGER);
      return true;
    }else if (path_type == TaskType::DETACH_CHARGER){
      LOG(ERROR) << "没有激活充电行为";
      return false;
    }

    if (path_type == TaskType::POINT_STAB)
    {
      LOG(INFO) << "Get POINT_STAB task";
      point_stab_executer_->SetPlan(orig_global_plan);
      state_manager_.SetState(LocalBehaviorState::POINT_STAB);
      LOG(INFO) << "设置LocalBehaviorState为POINT_STAB!";
      return true;
    }

    if (path_type == TaskType::TARGET_FOLLOW) // 20260804
    {
      LOG(INFO) << "Get TARGET_FOLLOW task,start follow";
      LOG(INFO) << "设置局部地图为默认参数";
      common_algorithm_->SetOriginalCostmapParam();
      state_manager_.SetState(LocalBehaviorState::TARGET_FOLLOW);
      target_follow_executer_->SetPlan();
      return true;
    }

    if (orig_global_plan.size() > 0)
    {
      double g_x = orig_global_plan.back().pose.position.x;
      double g_y = orig_global_plan.back().pose.position.y;
      double g_yaw = tf::getYaw(orig_global_plan.back().pose.orientation);
      LOG(INFO) << "IIMTPlannerROS::setPlan: get a global plan goal [px:" << g_x << ", py:" << g_y << ", pyaw:" << g_yaw << "]";
      PoseStampedVector tmp_global_plan = orig_global_plan;
      common_algorithm_->visualizer_util_->publishGlobalPlan(tmp_global_plan);
    }

    if (path_type == TaskType::SINGAL_GOAL || path_type == TaskType::CHARGE_GOAL)
    {
      LOG(INFO) << "IIMTPlannerROS:: SetPlan get a free nav path: " << (int)path_type;
      LOG(INFO) << "设置局部地图为默认参数";
      common_algorithm_->SetOriginalCostmapParam();
      free_decision_executer_->SetPlan(orig_global_plan);
      rotate_behavior_executer_->SetRotateRadius(true);
      state_manager_.SetState(LocalBehaviorState::FREE_PATH);
    }

    if (path_type == TaskType::FIX_FOLLOW || path_type == TaskType::AREA || path_type == TaskType::REMAINED || path_type == TaskType::EDGE_FOLLOW)
    {
      LOG(INFO) << "IIMTPlannerROS:: SetPlan get a fix nav path: " << (int)path_type;
      fix_follow_decision_executer_->SetPlan(orig_global_plan);
      if (path_type == TaskType::AREA){
        LOG(INFO) << "AREA设置局部地图为默认参数";
        common_algorithm_->SetOriginalCostmapParam();
        // 0就是fix_nav_controller_type_ 1就是area_clean_controller_type 3就是edge_clean_controller_type
        fix_follow_decision_executer_->SetAreaMoveModel(1);
      }else if(path_type == TaskType::FIX_FOLLOW || path_type == TaskType::REMAINED){
        LOG(INFO) << "FIX_FOLLOW or REMAINED设置局部地图为默认参数";
        common_algorithm_->SetOriginalCostmapParam();
        fix_follow_decision_executer_->SetAreaMoveModel(0);
      }else{ // 只能是贴边
        LOG(INFO) << "设置局部地图为贴边参数";
        common_algorithm_->SetEdgeCostmapParam();
        fix_follow_decision_executer_->SetAreaMoveModel(3);
      }
      rotate_behavior_executer_->SetRotateRadius(true);
      state_manager_.SetState(LocalBehaviorState::FIX_PATH);
    }
    if (path_type == TaskType::LINE_FOLLOW){
      LOG(INFO) << "IIMTPlannerROS:: SetPlan get a line follow nav path";
      LOG(INFO) << "设置局部地图为默认参数";
    }
    return true;
  }

  IIMTPlannerROS::~IIMTPlannerROS()
  {
    // delete dsrv_;// 20240930  先注释
  }

  // 局部控制器tc_ 指向这个函数计算局部控制速度
  NavStatusInfo IIMTPlannerROS::ComputeVel(geometry_msgs::Twist &cmd_vel)
  {
    boost::mutex::scoped_lock lock(configuration_mutex_);
    NavStatusInfo status;
    if (!UpdateRobotKinematics())
    {
      LOG(ERROR) << "Could not get robot pose";
      status.status = NavStatus::STATUS_ERROR;
      return status;
    }
    UpdateCostmap2D();

    auto state = state_manager_.GetState();
    // LOG(INFO) << "IIMTPlannerROS::ComputeVel state: " << (int)state;
    BehaviorBase *behavior_executer = behavior_manager_.GetBehavior(state);
    if (behavior_executer)
    {
      behavior_executer->ComputeVel(cmd_vel, status);
    }
    return status;
  }

  bool IIMTPlannerROS::UpdateRobotKinematics()
  {
    geometry_msgs::PoseStamped current_pose;
    if (!common_algorithm_->costmap_ros_->getRobotPose(current_pose))
    {
      return false;
    }
    behavior_manager_.UpdateRobotPose(current_pose);
    geometry_msgs::PoseStamped robot_vel;
    common_algorithm_->odom_helper_.getRobotVel(robot_vel);
    behavior_manager_.UpdateRobotVel(robot_vel);
    return true;
  }

  void IIMTPlannerROS::SetFirstFlag(bool flag)
  {
    behavior_manager_.SetFirstFlag(flag);
  }

  void IIMTPlannerROS::UpdateCostmap2D()
  {
    // Copy a costmap2d map to local for multi thread.
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(common_algorithm_->costmap_ros_->getCopiedCostmap()->getMutex()));
    // 注意:!!!: 这边多此一举
    common_algorithm_->costmap_2d_ = *common_algorithm_->costmap_ros_->getCopiedCostmap();
  }

  bool IIMTPlannerROS::isGoalReached()
  {
    boost::mutex::scoped_lock lock(configuration_mutex_);
    if (!isInitialized())
    {
      LOG(ERROR) << "This planner has not been initialized, please call initialize,before using this planner";
      return false;
    }

    auto state = state_manager_.GetState();
    BehaviorBase *behavior_executer = behavior_manager_.GetBehavior(state);
    if (behavior_executer)
    {
      return behavior_executer->IsGoalReached();
    }
    LOG(ERROR) << "没有这个状态，这个是错误的！！！" << (int)state;
    return true;
  }

  double IIMTPlannerROS::GetLeftDis()
  {
    auto state = state_manager_.GetState();
    BehaviorBase *behavior_executer = behavior_manager_.GetBehavior(state);
    if (behavior_executer)
    {
      return behavior_executer->GetLeftDis();
    }
    return 0.0;
  }

  double IIMTPlannerROS::ResetLeftDis()
  {
    auto state = state_manager_.GetState();
    BehaviorBase *behavior_executer = behavior_manager_.GetBehavior(state);
    if (behavior_executer)
    {
      behavior_executer->ResetLeftDis();
    }
    return 0.0;
  }
  void IIMTPlannerROS::MaxSpeedCallBack(const std_msgs::Float64 &msg)
  {
    LOG(INFO) << "set max_speed:" << msg.data;
    // fix_follow_decision_executer_->pp_controller_->max_follow_speed_ = msg.data; // 覆盖路径最大速度
  }
  bool IIMTPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    LOG(ERROR) << "IIMTPlannerROS::computeVelocityCommands is error.";
    return true;
  }
  // 没有用到任何信息
  bool IIMTPlannerROS::setNavParams(const robot_msgs::NavParamsConstPtr &msg)
  {
    boost::mutex::scoped_lock lock(configuration_mutex_);
    behavior_manager_.SetFootprint(common_algorithm_->costmap_ros_->getRobotFootprint());
    return true;
  }
  // void IIMTPlannerROS::reconfigureCB(IIMTPlannerConfig &config, uint32_t level)
  //{
  //   //default_config_ = config;
  //   LOG(INFO) << "IIMTPlannerROS::reconfigureCB Done,各个参数的值是：";
  //   LOG(INFO) << "max_vel_trans: " << config.max_vel_trans
  //             << ", min_vel_trans: " << config.min_vel_trans
  //             << ", max_vel_x: " << config.max_vel_x
  //             << ", min_vel_x: " << config.min_vel_x
  //             << ", max_vel_y: " << config.max_vel_y
  //             << ", min_vel_y: " << config.min_vel_y
  //             << ", max_vel_theta: " << config.max_vel_theta
  //             << ", min_vel_theta: " << config.min_vel_theta
  //             << ", acc_lim_x: " << config.acc_lim_x
  //             << ", acc_lim_y: " << config.acc_lim_y
  //             << ", acc_lim_theta: " << config.acc_lim_theta
  //             << ", acc_lim_trans: " << config.acc_lim_trans
  //             << ", prune_plan: " << config.prune_plan
  //             << ", xy_goal_tolerance: " << config.xy_goal_tolerance
  //             << ", yaw_goal_tolerance: " << config.yaw_goal_tolerance
  //             << ", trans_stopped_vel: " << config.trans_stopped_vel
  //             << ", theta_stopped_vel: " << config.theta_stopped_vel
  //             << ", sim_time: " << config.sim_time
  //             << ", minimum_sim_dis: " << config.minimum_sim_dis
  //             << ", sim_granularity: " << config.sim_granularity
  //             << ", angular_sim_granularity: " << config.angular_sim_granularity
  //             << ", path_distance_bias: " << config.path_distance_bias
  //             << ", goal_distance_bias: " << config.goal_distance_bias
  //             << ", occdist_scale: " << config.occdist_scale
  //             << ", twirling_scale: " << config.twirling_scale
  //             << ", stop_time_buffer: " << config.stop_time_buffer
  //             << ", oscillation_reset_dist: " << config.oscillation_reset_dist
  //             << ", oscillation_reset_angle: " << config.oscillation_reset_angle
  //             << ", forward_point_distance: " << config.forward_point_distance
  //             << ", max_scaling_factor: " << config.max_scaling_factor
  //             << ", xy_goal_precise_tolerance: " << config.xy_goal_precise_tolerance
  //             << ", vx_samples: " << config.vx_samples
  //             << ", vy_samples: " << config.vy_samples
  //             << ", vth_samples: " << config.vth_samples
  //             << ", use_dwa: " << config.use_dwa
  //             << ", restore_defaults: " << config.restore_defaults
  //             << ", get_local_plan_distance: " << config.get_local_plan_distance
  //             << ", check_obstacle_distance: " << config.check_obstacle_distance
  //             << ", wait_duration_for_obstacle: " << config.max_vel_trans
  //             << ", minimum_radius: " << config.minimum_radius
  //             << ", goal_dis_tolerance: " << config.goal_dis_tolerance
  //             << ", robot_model: " << config.robot_model;
  // }
  void IIMTPlannerROS::ReconfigureConfig(){
    boost::mutex::scoped_lock lock(configuration_mutex_);
    ros::NodeHandle private_nh("~/" + name_);
    all_config_.changeAllAreaParam(private_nh);
    if (all_config_.checkParamUpdate(private_nh)){
      LOG(WARNING) << "开始动态更改参数.....";
      behavior_manager_.ReconfigureConfig();
    }else{
      LOG(INFO) << "IIMTPlannerROS参数没有发生改变,不进行更改.";
    }
  }
  void IIMTPlannerROS::DynamicConfigChangeCallBack(const std_msgs::BoolConstPtr &msg) {
    ReconfigureConfig();
  }
};
