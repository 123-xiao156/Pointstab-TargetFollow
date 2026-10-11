#ifndef GRACEFUL_CONTROLLER_H
#define GRACEFUL_CONTROLLER_H

#include <algorithm>
#include <list>
#include <cmath>
#include <angles/angles.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <common/common_algorithm.h>
#include <common/common_params.h>

namespace graceful_controller
{
class GracefulController
{
public:
  /**********************************************************************
   * Description: graceful控制器的构造函数
   * Input:     name：行为的命名空间
   *            default_config：所有的配置参数
   * Output:    common_algorithm：所有行为公用的算法接口
   * Return:    无
   * Modify Date    Version    Author          Modification
   * -----------------------------------------------
   * 2026/03/14     V1.0      邓同兴            代码初次注释
   **********************************************************************/
  GracefulController(const std::string &name, const LocalPlannerConfig &default_config, std::shared_ptr<CommonAlgorithm> &common_algorithm);
  /**********************************************************************
   * Description: 根据给定目标点计算速度
   * Input:     x：目标点在机器人坐标系下的x坐标
   *            y：目标点在机器人坐标系下的y坐标
   *            theta：目标点在机器人坐标系下的角度
   *            backward_motion：机器人是否应该向后移动
   * Output:    vel_x: 计算得到的线速度
   *            vel_th: 计算得到的角速度
   * Return:    计算成功返回true，否则返回false
   * Modify Date    Version    Author          Modification
   * -----------------------------------------------
   * 2026/03/14     V1.0      邓同兴            代码初次注释
   **********************************************************************/
  bool Approach(const double &x, const double &y, const double &theta, double &vel_x, double &vel_th, bool backward_motion = false);
  /**********************************************************************
   * Description: 计算需要执行的速度
   * Input:     current_pose：当前车的位姿
   *            current_robot_vel：当前车的速度
   *            transformed_plan：局部路径
   * Output:    cmd_vel：需要计算的速度
   * Return:    计算是否成功
   * Modify Date    Version    Author          Modification
   * -----------------------------------------------
   * 2026/03/14      V1.0      邓同兴            代码初次注释
   **********************************************************************/
  bool ComputeVel(geometry_msgs::PoseStamped &current_pose, geometry_msgs::PoseStamped &current_robot_vel, const PoseStampedVector &transformed_plan, geometry_msgs::Twist &cmd_vel);
  /**********************************************************************
   * Description: 设置局部规划器跟踪的路径
   * Input:     plan: 需要跟随的路径
   * Output:    filtered_plan: 过滤处理后的路径
   * Return:    路径是否设置成功
   * Modify Date    Version    Author          Modification
   * -----------------------------------------------
   * 2026/03/14      V1.0      邓同兴            代码初次注释
   **********************************************************************/
  bool SetPlan(const std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& filtered_plan);
  /**********************************************************************
   * Description: 旋转机器人朝向目标点
   * Input:     pose: base系下目标点的位姿
   * Output:    cmd_vel：计算出的速度
   * Return:    计算出的朝向差
   * Modify Date    Version    Author          Modification
   * -----------------------------------------------
   * 2026/03/14      V1.0      邓同兴            代码初次注释
   **********************************************************************/
  double RotateTowards(const geometry_msgs::PoseStamped& pose, geometry_msgs::Twist& cmd_vel);
  /**********************************************************************
   * Description: 计算机器人旋转指定角度的速度
   * Input:     yaw: 需要旋转的角度
   * Output:    cmd_vel：计算出的速度
   * Return:    无
   * Modify Date    Version    Author          Modification
   * -----------------------------------------------
   * 2026/03/14      V1.0      邓同兴            代码初次注释
   **********************************************************************/
  void RotateTowards(const double &yaw, geometry_msgs::Twist& cmd_vel);

private:
  /**********************************************************************
   * Description: 模拟路径
   * Input:     target_pose: 目标点
   * Output:    cmd_vel：计算出的速度
   * Return:    是否成功
   * Modify Date    Version    Author          Modification
   * -----------------------------------------------
   * 2026/03/14      V1.0      邓同兴            代码初次注释
   **********************************************************************/
  bool Simulate(const geometry_msgs::PoseStamped& target_pose, geometry_msgs::Twist& cmd_vel);
  /**********************************************************************
   * Description: 计算路径上每个点到机器人的距离
   * Input:     poses: 路径点
   * Output:    distances: 每个点到机器人的距离
   * Return:    无
   * Modify Date    Version    Author          Modification
   * -----------------------------------------------
   * 2026/03/14      V1.0      邓同兴            代码初次注释
   **********************************************************************/
  void ComputeDistanceAlongPath(const std::vector<geometry_msgs::PoseStamped>& poses, std::vector<double>& distances);

  ros::Publisher target_pose_pub_;
  std::shared_ptr<CommonAlgorithm> &common_algorithm_;
  const LocalPlannerConfig &default_config_;
  double max_abs_velocity_;
  bool goal_tolerance_met_; // 曾经到达过目标点
  bool has_new_path_;
};
}  // namespace graceful_controller

#endif  // GRACEFUL_CONTROLLER_H
