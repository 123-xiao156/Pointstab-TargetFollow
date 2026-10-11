#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H
#define DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <dynamic_reconfigure/server.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <robot_msgs/LocalPlannerStatus.h>
#include <iimt_local_planner/IIMTPlannerConfig.h>

#include "common/common_algorithm.h"
#include "common/common_params.h"
#include "common/state_manager.h"


#include "behavior/behavior_base.h"
#include "behavior/behavior_idle.h"
#include "behavior/behavior_manager.h"
#include "behavior/behavior_fix_follow.h"
#include "behavior/behavior_free_nav.h"
#include "behavior/behavior_rotate.h"
#include "behavior/behaviors_recovery.h"
#include "behavior/behavior_relocalization.h"
#include "behavior/behavior_charging.h"
#include "behavior/behavior_point_stab.h"
#include "behavior/behavior_target_follow.h"
// #include "behavior/behavior_line_follow.h"


#include "controller/dwa_controller.h"
#include "controller/mpc_controller.h"
#include "controller/teb_local_planner/teb_local_planner_controller.h"

namespace iimt_local_planner {

class IIMTPlannerROS : public nav_core::BaseLocalPlanner {
 public:
   IIMTPlannerROS();
  ~IIMTPlannerROS();
  /**********************************************************************
  * Description: 局部规划器初始化
  * Input:     name: 局部规划器名称
  *            tf: tf树
  * Output:    costmap_ros: 外部costmap，可以修改相关函数
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros);
  /**********************************************************************
  * Description: 老框架接口，局部规划器核心函数，即计算速度
  * Input: 
  * Output:    cmd_vel：计算的速度信息
  *            plan：使用的局部规划路径
  * Return:    计算成功与否
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;
  /**********************************************************************
  * Description: 新框架接口，局部规划器核心函数，即计算速度
  * Input: 
  * Output:    cmd_vel：计算的速度信息
  *            plan：使用的局部规划路径
  * Return:    状态属性
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  NavStatusInfo ComputeVel(geometry_msgs::Twist &cmd_vel) override;
  /**********************************************************************
  * Description: 老框架接口，task_base设置全局路径，并对行为进行初始化
  * Input:     orig_global_plan: 全局路径
  * Output:    无
  * Return:    设置成功与否
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  bool setPlan(const PoseStampedVector &orig_global_plan) override {
    return true;
  };
  /**********************************************************************
  * Description: task_base设置全局路径，并对行为进行初始化
  * Input:     orig_global_plan: 全局路径
  *            path_type       : 属于什么任务
  * Output:    无
  * Return:    设置成功与否
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  bool setPlan(const PoseStampedVector &orig_global_plan, TaskType path_type) override;
  /**********************************************************************
  * Description: 设置导航参数
  * Input:     msg: 导航参数
  * Output:    无
  * Return:    设置成功与否
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  bool setNavParams(const robot_msgs::NavParamsConstPtr &msg) override;
  /**********************************************************************
  * Description: 获取设置的最大速度
  * Input:     msg: 最大速度
  * Output:    无
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  void MaxSpeedCallBack(const std_msgs::Float64 &msg);
  /**********************************************************************
  * Description: 重置各个行为的剩余距离
  * Input:     无
  * Output:    无
  * Return:    返回0.0
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  double ResetLeftDis() override;
  /**********************************************************************
  * Description: 得到任务剩余的距离
  * Input:     无
  * Output:    无
  * Return:    返回任务剩余的距离
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  double GetLeftDis() override;
  /**********************************************************************
  * Description: 设置重新计时标志
  * Input:     flag: 是否重新计时
  * Output:    无
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0      dtx            代码初次注释
  **********************************************************************/
  void SetFirstFlag(bool flag) override;
  /**********************************************************************
  * Description: 返回变量initialized_
  * Input:     无
  * Output:    无
  * Return:    返回变量initialized_
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  bool isInitialized() { return initialized_; }
  /**********************************************************************
  * Description: 返回是否到达目标点
  * Input:     无
  * Output:    无
  * Return:    返回是否到达目标点
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  bool isGoalReached() override;
  /**********************************************************************
  * Description: 获取设置的最大速度
  * Input:     msg: 最大速度
  * Output:    无
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  void DynamicConfigChangeCallBack(const std_msgs::BoolConstPtr &msg);
 private:
  /**********************************************************************
  * Description: 动态参数配置
  * Input:     config：需要更改的参数
  *            level：无用值
  * Output:    无
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  void reconfigureCB(IIMTPlannerConfig &config, uint32_t level);
  void ReconfigureConfig();
  /**********************************************************************
  * Description: 从costmap中得到地图，主要是防止规划的是否抢锁，导致costmap不更新
  * Input:     无
  * Output:    无
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  void UpdateCostmap2D();
  /**********************************************************************
  * Description: 得到机器人位姿和速度，然后更新到各个行为树
  * Input:     无
  * Output:    无
  * Return:    更新是否成功
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  bool UpdateRobotKinematics();

 private:
  bool initialized_;
  std::string name_;
  ros::Subscriber dynamic_speed_sub_, dynamic_config_change_;
  LocalPlannerConfig all_config_;
  boost::mutex configuration_mutex_;
  std::shared_ptr<CommonAlgorithm> common_algorithm_;

  StateManager state_manager_;

  std::shared_ptr<dwa_local_planner::DWAPlanner> dwa_controller_;
  std::shared_ptr<mpc_controller::MPCController> mpc_controller_;
  std::shared_ptr<teb_local_planner::TebLocalPlannerROS> teb_controller_;

  BehaviorManager behavior_manager_;
  std::shared_ptr<BehaviorIdle> idle_behavior_executer_;
  std::shared_ptr<RotateBehavior> rotate_behavior_executer_;
  std::shared_ptr<BehaviorFixNav> fix_follow_decision_executer_;
  std::shared_ptr<BehaviorFreeNav> free_decision_executer_;
  std::shared_ptr<RecoveryBehaviors> recovery_behavior_executer_;
  std::shared_ptr<RelocBehaviorExecuter> reloc_behavior_executer_;
  std::shared_ptr<ChargingBehavior> charge_behavior_executer_;
  std::shared_ptr<PointStabBehavior> point_stab_executer_;
  std::shared_ptr<BehaviorTargetFollow> target_follow_executer_; // 20260804

  //dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_; // 20240930   先注释
  //dwa_local_planner::DWAPlannerConfig default_config_; // 20240930   先注释
};

}  // namespace iimt_local_planner

#endif
