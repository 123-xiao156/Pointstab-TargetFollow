#include "behavior/behavior_target_follow.h"
BehaviorTargetFollow::BehaviorTargetFollow(std::string &name, const LocalPlannerConfig &default_config, std::shared_ptr<CommonAlgorithm> &common_algorithm)
    : BehaviorBase(),
      default_config_(default_config),
      common_algorithm_(common_algorithm),
      obstacle_costs_(common_algorithm->costmap_ros_->getCostmap()),
      goal_costs_(common_algorithm->costmap_ros_->getCostmap(), 0.0, 0.0, true),
      goal_front_costs_(common_algorithm->costmap_ros_->getCostmap(), 0.0, 0.0, true)
{
    traj_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("traj_cloud", 1000); // 轨迹云发布器
    target_sub_ = nh_.subscribe("follow_target", 1, &BehaviorTargetFollow::TargetCallback, this);

    goal_front_costs_.setStopOnFailure(false);
    oscillation_costs_.resetOscillationFlags();
    obstacle_costs_.setSumScores(default_config_.dwa_params.sum_scores);

    std::vector<base_local_planner::TrajectoryCostFunction *> critics;
    critics.push_back(&oscillation_costs_);
    critics.push_back(&obstacle_costs_);
    critics.push_back(&goal_front_costs_);
    critics.push_back(&goal_costs_);
    critics.push_back(&twirling_costs_);

    std::vector<base_local_planner::TrajectorySampleGenerator *> generator_list;
    generator_list.push_back(&generator_);
    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    reconfigure(default_config_);
    state_map_fun_[ControlNavState::IDLE] = std::bind(&BehaviorTargetFollow::ExecuteIdle, this, std::placeholders::_1, std::placeholders::_2);
    state_map_fun_[ControlNavState::FOLLOW_TARGET_STATE] = std::bind(&BehaviorTargetFollow::ExecuteFollowTarget, this, std::placeholders::_1, std::placeholders::_2);
    CreateFsmState();

    LOG(INFO) << "                           follow_max_retry_time: " << default_config_.targetfollow_params.follow_max_retry_time;
    LOG(INFO) << "                           follow_max_vel_trans: " << default_config_.targetfollow_params.follow_max_vel_trans;
    LOG(INFO) << "                           follow_min_vel_trans: " << default_config_.targetfollow_params.follow_min_vel_trans;
    LOG(INFO) << "                           follow_max_vel_x: " << default_config_.targetfollow_params.follow_max_vel_x;
    LOG(INFO) << "                           follow_min_vel_x: " << default_config_.targetfollow_params.follow_min_vel_x;
    LOG(INFO) << "                           follow_max_vel_y: " << default_config_.targetfollow_params.follow_max_vel_y;
    LOG(INFO) << "                           follow_min_vel_y: " << default_config_.targetfollow_params.follow_min_vel_y;
    LOG(INFO) << "                           follow_max_vel_theta: " << default_config_.targetfollow_params.follow_max_vel_theta;
    LOG(INFO) << "                           follow_acc_lim_x: " << default_config_.targetfollow_params.follow_acc_lim_x;
    LOG(INFO) << "                           follow_acc_lim_y: " << default_config_.targetfollow_params.follow_acc_lim_y;
    LOG(INFO) << "                           follow_acc_lim_theta: " << default_config_.targetfollow_params.follow_acc_lim_theta;
    LOG(INFO) << "                           follow_safe_distance: " << default_config_.targetfollow_params.follow_safe_distance;
    LOG(INFO) << "                           follow_move_threshold: " << default_config_.targetfollow_params.follow_move_threshold;
    LOG(INFO) << "                           follow_static_time: " << default_config_.targetfollow_params.follow_static_time;
    LOG(WARNING) << "BehaviorTargetFollow初始化成功";
}
void BehaviorTargetFollow::TargetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    if (std::isnan(p.x) || std::isnan(p.y) || std::isinf(p.x) || std::isinf(p.y))
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        target_pose_ = *msg;
        has_valid_target_data_ = true;
    }
}
void BehaviorTargetFollow::ReconfigureConfig()
{
    LOG(WARNING) << "BehaviorTargetFollow::ReconfigureConfig()";
    reconfigure(default_config_);
}
void BehaviorTargetFollow::reconfigure(const LocalPlannerConfig &config)
{
    std::lock_guard<std::mutex> config_lock(mutex_);
    double sim_period = 1.0 / config.common_params.controller_frequency;
    generator_.setParameters(config.dwa_params.sim_time, config.dwa_params.sim_granularity, config.dwa_params.angular_sim_granularity, config.dwa_params.use_dwa, sim_period);

    goal_costs_.setScale(common_algorithm_->costmap_ros_->getCostmap()->getResolution() * config.dwa_params.goal_distance_bias);
    obstacle_costs_.setScale(config.dwa_params.occdist_scale);
    oscillation_costs_.setOscillationResetDist(config.dwa_params.oscillation_reset_dist, config.dwa_params.oscillation_reset_angle);
    goal_front_costs_.setScale(common_algorithm_->costmap_ros_->getCostmap()->getResolution() * config.dwa_params.goal_distance_bias);
    goal_front_costs_.setXShift(config.dwa_params.forward_point_distance);
    obstacle_costs_.setParams(config.robot.max_vel_trans, config.dwa_params.max_scaling_factor, config.dwa_params.scaling_speed);
    twirling_costs_.setScale(config.dwa_params.twirling_scale);

    vsamples_[0] = config.dwa_params.vx_samples;
    vsamples_[1] = config.dwa_params.vy_samples;
    vsamples_[2] = config.dwa_params.vth_samples;
}
void BehaviorTargetFollow::ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
    status.status = NavStatus::STATUS_RUNNING;
    auto state = state_machine_.GetState();
    ControlNavEvent event;
    if (state_map_fun_.find(state) != state_map_fun_.end())
    {
        event = state_map_fun_[state](cmd_vel, status);
    }
    state_machine_.Command(event);
}
void BehaviorTargetFollow::SetPlan()
{
    SetGoalReached(false);
    LOG(INFO) << "BehaviorTargetFollow::SetPlan...";
    state_machine_.SetState(ControlNavState::IDLE);
}
void BehaviorTargetFollow::CreateFsmState()
{
    LOG(WARNING) << "BehaviorTargetFollow::CreateFsmState()...";
    // IDLE状态-->FOLLOW_TARGET_STATE状态
    state_machine_.On(ControlNavState::IDLE, ControlNavEvent::REQUEST_FOLLOW_TARGET) = [&]
    {
        LOG(INFO) << "CreateFsmState: Request from IDLE to FOLLOW_TARGET_STATE";
        state_machine_.SetState(ControlNavState::FOLLOW_TARGET_STATE);
    };
}
ControlNavEvent BehaviorTargetFollow::ExecuteIdle(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
    LOG(INFO) << "BehaviorTargetFollow::ExecuteIDLE()...";
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_valid_target_data_)
    {
        LOG_EVERY_N(WARNING, 10) << "BehaviorTargetFollow::ExecuteIdle: target not received";
        return ControlNavEvent::NONE;
    }
    return ControlNavEvent::REQUEST_FOLLOW_TARGET;
}
ControlNavEvent BehaviorTargetFollow::ExecuteFollowTarget(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)
{
    LOG(INFO) << "BehaviorTargetFollow::ExecuteFollowTarget()...";
    static int count = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 第一次收到目标
        if (!has_last_target_)
        {
            current_goal_ = target_pose_;
            last_pose_ = current_goal_;
            has_last_target_ = true;
        }
        // 缓存目标点
        goal_queue_.push(target_pose_);
        if (goal_queue_.size() > goal_queue_size_)
        {
            goal_queue_.pop();
        }
        ChooseGoal();
    }
    if (++count % (int)(5 * default_config_.targetfollow_params.follow_static_time) == 0)
    {
        count = 0;
        double distance = dwa_math_util::CalDistM(target_pose_, last_pose_);
        if (distance < default_config_.targetfollow_params.follow_move_threshold) // 指定时间内的变化小于阈值，则认为完成跟随
        {
            LOG(INFO) << "Target has not moved for a long time, finish follow";
            SetGoalReached(true);
            SetZeroSpeed(cmd_vel);
            retry_count_ = 0;
            first_flag_ = true;
            has_last_target_ = false;
            has_valid_target_data_ = false;
            status.status = NavStatus::STATUS_REACHED_GOAL;
            return ControlNavEvent::NONE;
        }
        last_pose_ = target_pose_;
    }
    double dis = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        dis = dwa_math_util::CalDistM(target_pose_, current_pose_);
    }
    if (dis < default_config_.targetfollow_params.follow_safe_distance) // 目标距离机器人小于安全距离，则不跟随
    {
        if (first_flag_)
        {
            first_flag_ = false;
            start_time_ = ros::Time::now();
        }
        LOG_EVERY_N(WARNING, 5) << "Target distance is too close, not follow, time: " << ros::Time::now().toSec() - start_time_.toSec() << " s.";
        return ControlNavEvent::NONE;
    }
    else
    {
        first_flag_ = true;
        if (!ComputeWithDWA(current_goal_, cmd_vel))
        {
            retry_count_++;
            LOG(INFO) << "BehaviorTargetFollow::ExecuteFollowTarget: retry count: " << retry_count_;
            if (retry_count_ > default_config_.targetfollow_params.follow_max_retry_time)
            {
                has_valid_target_data_ = false;
                retry_count_ = 0;
                SetZeroSpeed(cmd_vel);
                has_last_target_ = false;
                status.status = NavStatus::STATUS_ERROR_IN_TARGET_FOLLOW;
            }
        }
        else
        {
            retry_count_ = 0;
        }
    }
    return ControlNavEvent::NONE;
}

bool BehaviorTargetFollow::ComputeWithDWA(const geometry_msgs::PoseStamped &goal, geometry_msgs::Twist &cmd_vel)
{
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = default_config_.base_frame_id;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    transformed_plan.push_back(goal);
    UpdatePlanAndLocalCosts(current_pose_, transformed_plan, common_algorithm_->costmap_ros_->getRobotFootprint());
    base_local_planner::Trajectory path = FindBestPath(current_pose_, goal, robot_vel_, drive_cmds);

    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf::getYaw(drive_cmds.pose.orientation);
    if (path.cost_ < 0)
    {
        LOG(ERROR) << "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.";
        return false;
    }
    return true;
}

void BehaviorTargetFollow::UpdatePlanAndLocalCosts(const geometry_msgs::PoseStamped &global_pose,
                                                   const std::vector<geometry_msgs::PoseStamped> &new_plan,
                                                   const std::vector<geometry_msgs::Point> &footprint_spec)
{
    obstacle_costs_.setFootprint(footprint_spec);
    goal_costs_.setTargetPoses(new_plan);

    geometry_msgs::PoseStamped goal_pose = new_plan.back();
    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));

    std::vector<geometry_msgs::PoseStamped> front_global_plan = new_plan;
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x + default_config_.dwa_params.forward_point_distance * cos(angle_to_goal);
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + default_config_.dwa_params.forward_point_distance * sin(angle_to_goal);

    goal_front_costs_.setTargetPoses(front_global_plan);
}

base_local_planner::Trajectory BehaviorTargetFollow::FindBestPath(const geometry_msgs::PoseStamped &global_pose,
                                                                  const geometry_msgs::PoseStamped &goal_pose,
                                                                  const geometry_msgs::PoseStamped &global_vel,
                                                                  geometry_msgs::PoseStamped &drive_velocities)
{
    std::lock_guard<std::mutex> config_lock(mutex_);

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));

    base_local_planner::LocalPlannerLimits limits;
    limits.max_vel_trans = default_config_.targetfollow_params.follow_max_vel_trans;
    limits.min_vel_trans = default_config_.targetfollow_params.follow_min_vel_trans;
    limits.max_vel_x = default_config_.targetfollow_params.follow_max_vel_x;
    limits.min_vel_x = default_config_.targetfollow_params.follow_min_vel_x;
    limits.max_vel_y = default_config_.targetfollow_params.follow_max_vel_y;
    limits.min_vel_y = -default_config_.targetfollow_params.follow_min_vel_y;
    limits.max_vel_theta = default_config_.targetfollow_params.follow_max_vel_theta;
    limits.min_vel_theta = -default_config_.targetfollow_params.follow_max_vel_theta;
    limits.acc_lim_x = default_config_.targetfollow_params.follow_acc_lim_x;
    limits.acc_lim_y = default_config_.targetfollow_params.follow_acc_lim_y;
    limits.acc_lim_theta = default_config_.targetfollow_params.follow_acc_lim_theta;

    generator_.initialise(pos, vel, goal, &limits, vsamples_);

    result_traj_.cost_ = -7;
    std::vector<base_local_planner::Trajectory> all_explored;
    scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);
    sensor_msgs::PointCloud2 traj_cloud;
    traj_cloud.header.frame_id = default_config_.map_frame;
    traj_cloud.header.stamp = ros::Time::now();

    sensor_msgs::PointCloud2Modifier cloud_mod(traj_cloud);
    cloud_mod.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                   sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32,
                                   "theta", 1, sensor_msgs::PointField::FLOAT32, "cost", 1,
                                   sensor_msgs::PointField::FLOAT32);

    unsigned int num_points = 0;
    // 在 behavior_target_follow.cpp:281 附近修改
    int total_traj = 0, valid_traj = 0;
    for (std::vector<base_local_planner::Trajectory>::iterator t = all_explored.begin(); t != all_explored.end(); ++t)
    {
        total_traj++;
        if (t->cost_ >= 0)
            valid_traj++;
        else
        {
            LOG(ERROR) << "Traj filtered: vx=%.3f, vy=%.3f, vth=%.3f, cost=%.1f" << t->xv_ << ", " << t->yv_ << ", " << t->thetav_ << ", " << t->cost_;
        }
    }
    LOG(ERROR) << "Total: " << total_traj << ", Valid: " << valid_traj;
    for (std::vector<base_local_planner::Trajectory>::iterator t = all_explored.begin(); t != all_explored.end(); ++t)
    {
        if (t->cost_ < 0)
            continue;
        num_points += t->getPointsSize();
    }

    cloud_mod.resize(num_points);
    sensor_msgs::PointCloud2Iterator<float> iter_x(traj_cloud, "x");
    for (std::vector<base_local_planner::Trajectory>::iterator t = all_explored.begin(); t != all_explored.end(); ++t)
    {
        if (t->cost_ < 0)
            continue;

        for (unsigned int i = 0; i < t->getPointsSize(); ++i)
        {
            double p_x, p_y, p_th;
            t->getPoint(i, p_x, p_y, p_th);
            iter_x[0] = p_x;
            iter_x[1] = p_y;
            iter_x[2] = 0.0;
            iter_x[3] = p_th;
            iter_x[4] = t->cost_;
            ++iter_x;
        }
    }
    traj_cloud_pub_.publish(traj_cloud);
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, default_config_.robot.min_vel_trans);

    if (result_traj_.cost_ < 0)
    {
        drive_velocities.pose.position.x = 0;
        drive_velocities.pose.position.y = 0;
        drive_velocities.pose.position.z = 0;
        drive_velocities.pose.orientation.w = 1;
        drive_velocities.pose.orientation.x = 0;
        drive_velocities.pose.orientation.y = 0;
        drive_velocities.pose.orientation.z = 0;
    }
    else
    {
        drive_velocities.pose.position.x = result_traj_.xv_;
        drive_velocities.pose.position.y = result_traj_.yv_;
        drive_velocities.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, result_traj_.thetav_);
        tf2::convert(q, drive_velocities.pose.orientation);
    }
    return result_traj_;
}

void BehaviorTargetFollow::ChooseGoal()
{
    double d = dwa_math_util::CalDistM(current_pose_, target_pose_);
    if (d <= 2.0)
    {
        current_goal_ = target_pose_;
        LOG_EVERY_N(INFO, 10) << "机器人当前距离跟随目标的距离小于2米 ";
    }
    else if (d < 3.0)
    {
        LOG_EVERY_N(INFO, 10) << "机器人当前距离跟随目标的距离在2米到3米之间 ";
        std::queue<geometry_msgs::PoseStamped> temp_q = goal_queue_;
        while (!temp_q.empty())
        {
            auto tmp_pose = temp_q.front();
            temp_q.pop();
            unsigned int nx, ny;
            common_algorithm_->costmap_ros_->getCostmap()->worldToMap(tmp_pose.pose.position.x, tmp_pose.pose.position.y, nx, ny);
            int cost = common_algorithm_->costmap_ros_->getCostmap()->getCost(nx, ny);
            if (cost < 250)
            {
                current_goal_ = tmp_pose;
                last_valid_goal_ = tmp_pose;
                // LOG(INFO) << "选择的缓存目标点为：" << tmp_pose.pose.position.x << " " << tmp_pose.pose.position.y << " " << tmp_pose.pose.position.z << " " << tmp_pose.pose.orientation.w << "cost = " << cost;
                break;
            }
        }
    }
    else
    {
        current_goal_ = last_valid_goal_;
        LOG_EVERY_N(INFO, 10) << "机器人当前距离跟随目标的距离大于3米 ";
    }
} 

