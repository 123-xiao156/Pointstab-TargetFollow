#ifndef BEHAVIOR_TARGET_FOLLOW_H
#define BEHAVIOR_TARGET_FOLLOW_H
#include <glog/logging.h>
#include <tf2/utils.h>
#include <queue>
#include <nav_core/fsm_base.h>
#include <base_local_planner/map_grid_visualizer.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/twirling_cost_function.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "common/common_msg.h"
#include "common/common_algorithm.h"
#include "common/common_params.h"
#include "behavior_base.h"
// #include "controller/dwa_controller.h"
class BehaviorTargetFollow : public BehaviorBase
{
public:
    /**********************************************************************
     * Description: 目标跟随行为的构造函数
     * Input:     default_config：所有的配置参数
     * Output:    common_algorithm：所有行为公用的算法接口
     * Return:    无
     **********************************************************************/
    BehaviorTargetFollow(std::string &name,
                         const LocalPlannerConfig &default_config,
                         std::shared_ptr<CommonAlgorithm> &common_algorithm);

    /**********************************************************************
     * Description: 目标跟随行为的析构函数
     * Input:     无
     * Output:    无
     * Return:    无
     **********************************************************************/
    ~BehaviorTargetFollow() = default;
    /**********************************************************************
     * Description: 设置路径
     * Input:     无
     * Output:    无
     * Return:    无
     **********************************************************************/
    void SetPlan();

private:
    /**********************************************************************
     * Description: 重新配置函数
     * Input:     config：新的配置参数
     * Output:    无
     * Return:    无
     **********************************************************************/
    void reconfigure(const LocalPlannerConfig &config);
    /**********************************************************************
     * Description: 跟随目标点回调函数
     * Input:     msg：目标点消息
     * Output:    无
     * Return:    无
     **********************************************************************/
    void TargetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    /**********************************************************************
     * Description: 计算速度
     * Input:     cmd_vel：速度消息
     * Output:    status：导航状态
     * Return:    无
     **********************************************************************/
    void ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
    /**********************************************************************
     * Description: 创建目标跟随行为状态机
     * Input:     无
     * Output:    无
     * Return:    无
     **********************************************************************/
    void CreateFsmState();
    /**********************************************************************
     * Description: 执行空闲状态
     * Input:     cmd_vel：速度消息
     * Output:    status：导航状态
     * Return:    无
     **********************************************************************/
    ControlNavEvent ExecuteIdle(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
    /**********************************************************************
     * Description: 执行准备跟随状态
     * Input:     cmd_vel：速度消息
     * Output:    status：导航状态
     * Return:    无
     **********************************************************************/
    // ControlNavEvent ExecutePrepareFollow(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
    /**********************************************************************
     * Description: 执行跟随目标点状态
     * Input:     cmd_vel：速度消息
     * Output:    status：导航状态
     * Return:    无
     **********************************************************************/
    ControlNavEvent ExecuteFollowTarget(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
    /**********************************************************************
     * Description: 计算速度
     * Input:     goal：目标点
     * Output:    cmd_vel：速度消息
     * Return:    无
     **********************************************************************/
    bool ComputeWithDWA(const geometry_msgs::PoseStamped &goal, geometry_msgs::Twist &cmd_vel);
    /**********************************************************************
     * Description: 更新路径和本地成本
     * Input:     global_pose：机器人全局位姿
     * new_plan：  局部路径
     * footprint_spec：机器人轮廓
     * Output:    无
     * Return:    无
     **********************************************************************/
    void UpdatePlanAndLocalCosts(const geometry_msgs::PoseStamped &global_pose,
                                  const std::vector<geometry_msgs::PoseStamped> &new_plan,
                                  const std::vector<geometry_msgs::Point> &footprint_spec);
    /**********************************************************************
     * Description: 查找最佳路径
     * Input:     global_pose：机器人全局位姿
     * goal_pose：目标点
     * global_vel：机器人当前速度
     * Output:    drive_velocities：计算得到的速度
     * Return:    最佳路径
     **********************************************************************/
    base_local_planner::Trajectory FindBestPath(const geometry_msgs::PoseStamped &global_pose,
                                                const geometry_msgs::PoseStamped &goal_pose,
                                                const geometry_msgs::PoseStamped &global_vel,
                                                geometry_msgs::PoseStamped &drive_velocities);  

    /**********************************************************************
     * Description: 选择DWA目标点
     * Input:     无
     * Output:    无
     * Return:    无
     **********************************************************************/                                    
    void ChooseGoal();
    /**********************************************************************
     * Description: 更新各个模块的参数
     * Input:     无
     * Output:    无
     * Return:    无
     **********************************************************************/
    void ReconfigureConfig();

private:
    int retry_count_ = 0; // 重试次数计数器
    int goal_queue_size_ = 10; // 目标点队列大小
    bool first_flag_{true};
    bool has_last_target_{false}; // 是否有上一个目标点
    std::atomic<bool> has_valid_target_data_{false}; // 是否有有效目标点数据
    double path_distance_bias_;
    double goal_distance_bias_;
    double occdist_scale_;

    ros::NodeHandle nh_;
    ros::Time start_time_;
    ros::Subscriber target_sub_;
    ros::Publisher traj_cloud_pub_; // 轨迹云发布器
    geometry_msgs::PoseStamped target_pose_; // 实时订阅到的目标位姿
    geometry_msgs::PoseStamped last_pose_; // 上一控制周期订阅到的目标位姿
    geometry_msgs::PoseStamped current_goal_; // 当前发送给DWA的目标点
    geometry_msgs::PoseStamped last_valid_goal_; // 冻结备份点，用于处理目标点不可达的情况
    std::queue<geometry_msgs::PoseStamped> goal_queue_; // 缓存目标点

    std::shared_ptr<CommonAlgorithm> &common_algorithm_;
    const LocalPlannerConfig &default_config_;
    std::mutex mutex_;

    Eigen::Vector3f vsamples_;

    base_local_planner::Trajectory result_traj_;
    base_local_planner::SimpleTrajectoryGenerator generator_;
    base_local_planner::OscillationCostFunction oscillation_costs_;
    base_local_planner::ObstacleCostFunction obstacle_costs_;
    base_local_planner::MapGridCostFunction goal_costs_;
    base_local_planner::MapGridCostFunction goal_front_costs_;
    base_local_planner::TwirlingCostFunction twirling_costs_;
    base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
    
    FsmMachine<ControlNavState, ControlNavEvent> state_machine_;
    std::map<ControlNavState, std::function<ControlNavEvent(geometry_msgs::Twist &, NavStatusInfo &)>> state_map_fun_;
};
#endif