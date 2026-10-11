#ifndef BEHAVIOR_POINT_STAB_H
#define BEHAVIOR_POINT_STAB_H
#include <nav_core/fsm_base.h>
#include "common/common_msg.h"
#include "common/common_algorithm.h"
#include "common/common_params.h"
#include "common/path_manager.h"
#include "common/visualizer_util.h"

#include "behavior/behavior_base.h"
#include "behavior/behavior_manager.h"
#include  "behavior/behavior_rotate.h"
#include "common/state_manager.h"
#include "controller/graceful_controller/graceful_controller.h"
class PointStabBehavior : public BehaviorBase
{
public:
  /**********************************************************************
  * Description: 点镇定行为的构造函数
  * Input:     default_config：所有的配置参数
  * Output:    common_algorithm：所有行为公用的算法接口
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2026/01/20      V1.0     邓同兴            代码初次注释
  **********************************************************************/
  PointStabBehavior(
      std::string &name,
      std::shared_ptr<CommonAlgorithm> &common_algorithm, 
      const LocalPlannerConfig &default_config);
  /**********************************************************************
  * Description: 计算需要执行的速度
  * Input:     无
  * Output:    cmd_vel：需要计算的速度
  *            status：行为执行状态
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2026/01/20      V1.0     邓同兴            代码初次注释
  **********************************************************************/
  void ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) override;
  /**********************************************************************
  * Description: 设置全局路径
  * Input:     plan：目标点
  * Output:    无
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2026/01/20      V1.0     邓同兴            代码初次注释
  **********************************************************************/
  void SetPlan(const PoseStampedVector &plan);
  /**********************************************************************
  * Description: 将充电行为轮廓设置到collision_detector_中
  * Input:     footprint_spec：需要设置的轮廓
  * Output:    无
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2026/01/20      V1.0     邓同兴            代码初次注释
  **********************************************************************/
  void SetFootprint(const std::vector<geometry_msgs::Point> &footprint_spec) override
  {
    if (collision_detector_)
    {
      collision_detector_->SetFootprint(footprint_spec);
      LOG(WARNING) << "PointStabilizationBehavior 碰撞检测车体轮廓初始化成功。";
    }
  }
  /**********************************************************************
  * Description: 更新各个模块的参数
  * Input:     config: 需要更改的参数
  * Output:    无
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2026/01/20      V1.0     邓同兴            代码初次注释
  **********************************************************************/
  void ReconfigureConfig() override;

  /**********************************************************************
   * Description: 执行idle状态
   * Input:     无
   * Output:    cmd_vel：计算需要的速度
   *            status：当前状态下的执行情况
   * Return:    状态机跳转情况
   * Modify Date    Version    Author          Modification
   * -----------------------------------------------
   * 2026/01/20      V1.0      邓同兴            代码初次注释
   **********************************************************************/
  ControlNavEvent ExecIdle(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  /**********************************************************************
  * Description: 执行平滑控制状态
  * Input:     无
  * Output:    cmd_vel：计算需要的速度
  *            status：当前状态下的执行情况
  * Return:    状态机跳转情况
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2026/01/20      V1.0      邓同兴            代码初次注释
  **********************************************************************/
  ControlNavEvent ExecGracefulControl(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  /**********************************************************************
  * Description: 执行rotationinplace状态
  * Input:     无
  * Output:    cmd_vel：计算需要的速度
  *            status：当前状态下的执行情况
  * Return:    状态机跳转情况
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2026/01/20      V1.0      邓同兴            代码初次注释
  **********************************************************************/
  ControlNavEvent ExecRotationInPlace(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  /**********************************************************************
  * Description: 执行movetogoal状态
  * Input:     无
  * Output:    cmd_vel：计算需要的速度
  *            status：当前状态下的执行情况
  * Return:    状态机跳转情况
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2026/01/20      V1.0      邓同兴            代码初次注释
  **********************************************************************/
  ControlNavEvent ExecMoveToGoal(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  /**********************************************************************
  * Description: 执行rotationatgoal状态
  * Input:     无
  * Output:    cmd_vel：计算需要的速度
  *            status：当前状态下的执行情况
  * Return:    状态机跳转情况
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2026/01/20      V1.0      邓同兴            代码初次注释
  **********************************************************************/
  ControlNavEvent ExecRotationAtGoal(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);

  /**********************************************************************
  * Description: 旋转到目标角度
  * Input:     goal_angle：目标角度
  *            cmd_vel:计算的速度
  * Output:    start_th：开始的角度信息
  * Return:    旋转状态
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  RotationState RotateToGoal(double goal_angle, geometry_msgs::Twist &cmd_vel);

  /**********************************************************************
  * Description: 正向、反向为相对计算出的速度方向
  *               1、正向旋转检查成功，则继续旋转，下一次进来继续正向旋转检查
  *               2、正向旋转检查失败，反向旋转检查成功后，下一次进来只会继续反向旋转检查，如果失败直接清零重置
  *               3、反向旋转检查失败，直接清零重置
  * Input:     global_pose：当前位姿
  *            robot_vel：计算出来的速度
  *            max_vel_theta
  *            min_vel_theta
  * Output:    goal_th：
  *            cmd_vel: 机器人计算速度
  * Return:    计算是否成功
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  bool CheckRotateToGoalPossibilityAndCalcVel(
      const geometry_msgs::PoseStamped &global_pose, const geometry_msgs::PoseStamped &robot_vel,
      const double &max_vel_theta, const double &min_vel_theta,double goal_th,
      geometry_msgs::Twist &cmd_vel);

  /**********************************************************************
  * Description: 检查从robot_vel到v_theta_samp速度，机器人会不会碰撞
  * Input:     global_pose：当前位姿
  *            robot_vel：计算出来的速度
  * Output:    v_theta_samp：目标速度
  * Return:    检查从robot_vel到v_theta_samp速度，机器人会不会碰撞
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2025/11/10      V1.0                   代码初次注释
  **********************************************************************/
  bool IsRotationValid(const geometry_msgs::PoseStamped &global_pose, const geometry_msgs::PoseStamped &robot_vel, double v_theta_samp);

private:
  /**********************************************************************
  * Description: 创建点镇定行为的状态机
  * Input:     无
  * Output:    无
  * Return:    无
  * Modify Date    Version    Author          Modification
  * -----------------------------------------------
  * 2026/01/20      V1.0     邓同兴            代码初次注释
  **********************************************************************/
  void CreateFsmState();

private:
  bool reverse_rotation_check_ = false; // 是否正在进行过反向旋转了
  bool is_positive_ = true;             // 记录反向旋转执行的时候，角速度是逆时针吗
  ros::Time meet_obstacle_start_time_ = ros::Time(0);
  geometry_msgs::PoseStamped goal_pose_;
  std::vector<geometry_msgs::PoseStamped> filtered_plan_;

  const LocalPlannerConfig &default_config_;
  std::unique_ptr<CostmapCollisionDetector> collision_detector_;
  std::shared_ptr<CommonAlgorithm> &common_algorithm_;
  std::shared_ptr<graceful_controller::GracefulController> graceful_controller_;

  FsmMachine<ControlNavState, ControlNavEvent> state_machine_;
  std::map<ControlNavState, std::function<ControlNavEvent(geometry_msgs::Twist &, NavStatusInfo &)>> state_map_fun_;
};

#endif