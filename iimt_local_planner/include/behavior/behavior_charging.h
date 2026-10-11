#ifndef BEHAVIOR_CHARGING_H
#define BEHAVIOR_CHARGING_H
#include <nav_core/fsm_base.h>

#include "common/common_msg.h"
#include "common/common_algorithm.h"
#include "common/common_params.h"
#include "common/path_manager.h"
#include "common/visualizer_util.h"
#include "common/state_manager.h"

#include "behavior/behavior_base.h"
#include "behavior/behavior_manager.h"
#include "behavior/behavior_rotate.h"

#include "controller/charge_controller/charge_controller.h"

class ChargingBehavior : public BehaviorBase
{
public:
  /**********************************************************************
   * Description: 充电行为的构造函数
   * Input:     default_config：所有的配置参数
   * Output:    common_algorithm：所有行为公用的算法接口
   *            rotate_behavior：旋转行为
   * Return:    无
   **********************************************************************/
  ChargingBehavior(std::shared_ptr<CommonAlgorithm> &common_algorithm, const LocalPlannerConfig &default_config);
  /**********************************************************************
   * Description: 计算需要执行的速度
   * Input:     无
   * Output:    cmd_vel：需要计算的速度
   *            status：行为执行状态
   * Return:    无
   **********************************************************************/
  void ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) override;
  /**********************************************************************
   * Description: 设置全局路径
   * Input:     task_type：充电任务类型
   * Output:    无
   * Return:    无
   **********************************************************************/
  void SetPlan(TaskType task_type);
  /**********************************************************************
   * Description: 将充电行为轮廓设置到collision_detector_中
   * Input:     footprint_spec：需要设置的轮廓
   * Output:    无
   * Return:    无
   **********************************************************************/
  void SetFootprint(const std::vector<geometry_msgs::Point> &footprint_spec) override
  {
    LOG(WARNING) << "ChargingBehavior不使用外部碰撞检测。";
  }
  void SetFirstFlag(bool flag) final
  {
    charge_controller_->first_flag_ = flag;
  }
  /**********************************************************************
   * Description: 更新各个模块的参数
   * Input:     config: 需要更改的参数
   * Output:    无
   * Return:    无
   **********************************************************************/
  void ReconfigureConfig() override;

private:
  /**********************************************************************
   * Description: 执行idle状态
   * Input:     无
   * Output:    cmd_vel：计算需要的速度
   *            status：当前状态下的执行情况
   * Return:    状态机跳转情况
   **********************************************************************/
  ChargeBehaviorEvent ExecIDLE(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  /**********************************************************************
   * Description: 执行开始充电状态，包含对桩和重试
   * Input:     无
   * Output:    cmd_vel：计算需要的速度
   *            status：当前状态下的执行情况
   * Return:    状态机跳转情况
   **********************************************************************/
  ChargeBehaviorEvent ExecStartCharge(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  /**********************************************************************
   * Description: 执行结束充电状态，即退桩
   * Input:     无
   * Output:    cmd_vel：计算需要的速度
   *            status：当前状态下的执行情况
   * Return:    状态机跳转情况
   **********************************************************************/
  ChargeBehaviorEvent ExecEndCharge(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  /**********************************************************************
   * Description: 创建充电行为的状态机
   * Input:     无
   * Output:    无
   * Return:    无
   **********************************************************************/
  void CreateFsmState();

private:
  int count_ = 0;
  TaskType current_task_type_;
  const LocalPlannerConfig &default_config_;
  std::shared_ptr<CommonAlgorithm> &common_algorithm_;
  std::shared_ptr<charge_controller::ChargeController> charge_controller_;

  FsmMachine<ChargeBehaviorState, ChargeBehaviorEvent> state_machine_;
  std::map<ChargeBehaviorState, std::function<ChargeBehaviorEvent(geometry_msgs::Twist &, NavStatusInfo &)>> state_map_fun_;
};

#endif
