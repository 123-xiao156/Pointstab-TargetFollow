#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace CHARGE_ODT
{
    struct Params
    {
        int min_obstacle_points_num; // 最小障碍点数
        double judge_width;
        double judge_depth;
        double judge_back_depth;
        Eigen::Matrix4d T_baselink_to_lidar;
    };

    struct LaserPoint
    {
        Eigen::Vector3f position;
    };

    class ObstacleDetect
    {

    public:
        Params params_;

        /**********************************************************
         * Description: 障碍物检测类含参构造函数，初始化相关成员变量
         * Input:    param: 障碍物检测相关参数
         * Output:   无
         * Return:   无
         * Modify Date       Version       Author       Modification    
         * ---------------------------------------------------------
         * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
         **********************************************************/
        ObstacleDetect(Params &param) : params_(param){}
        
        /**********************************************************
         * Description: 判断当前点是否在检测区域内
         * Input:    x: 当前点的x坐标
         *           y: 当前点的y坐标
         * Output:   无
         * Return:   true: 在检测区域内
         *           false: 不在检测区域内
         * Modify Date       Version       Author       Modification
         * ---------------------------------------------------------
         * 2026/01/30        v1.0          liuzhiwen    代码初次注释
         **********************************************************/
        inline bool IsInJudgeRegion(double x, double y)
        {
            return (x >= params_.judge_back_depth && x <= params_.judge_depth) &&
                   (std::abs(y) <= (params_.judge_width / 2.0));
        }

        /**********************************************************
         * Description: 判断是否存在障碍
         * Input:    msg: sensor_msgs::PointCloud2消息指针
         * Output:   无
         * Return:   true: 存在障碍
         *           false: 不存在障碍
         * Modify Date       Version       Author       Modification
         * ---------------------------------------------------------
         * 2026/01/30        v1.0          liuzhiwen    代码初次注释
         **********************************************************/
        bool JudgeFrontObstacle(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            // 转成 PCL 的中间格式
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);

            // 再转成带模板的 pcl::PointCloud
            pcl::PointCloud<pcl::PointXYZL>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZL>);
            pcl::fromPCLPointCloud2(pcl_pc2, *clouds);

            size_t lidar_cnt = 0, camera_cnt = 0, ultra_cnt = 0;
            bool has_lidar_obs = false, has_camera_obs = false, has_ultra_obs = false;

            for (const auto &pt : clouds->points)
            {
                double x_raw = pt.x;
                double y_raw = pt.y;

                double x_base = params_.T_baselink_to_lidar(0, 0) * x_raw +
                                params_.T_baselink_to_lidar(0, 1) * y_raw +
                                params_.T_baselink_to_lidar(0, 3);
                double y_base = params_.T_baselink_to_lidar(1, 0) * x_raw +
                                params_.T_baselink_to_lidar(1, 1) * y_raw +
                                params_.T_baselink_to_lidar(1, 3);

                if (pt.label == 0) // lidar
                {
                    if (!has_lidar_obs && IsInJudgeRegion(x_base, y_base))
                    {
                        lidar_cnt++;
                        if (lidar_cnt > params_.min_obstacle_points_num)
                        {
                            has_lidar_obs = true;
                            break;
                        }
                    }
                }
                else if (pt.label == 1) // camera
                {
                    if (!has_camera_obs && IsInJudgeRegion(x_base, y_base))
                    {
                        camera_cnt++;
                        if (camera_cnt > params_.min_obstacle_points_num)
                        {
                            has_camera_obs = true;
                            break;
                        }
                    }
                }
                else if (pt.label == 2) // ultrasound
                {
                    if (!has_ultra_obs && IsInJudgeRegion(x_base, y_base))
                    {
                        ultra_cnt++;
                        if (ultra_cnt > params_.min_obstacle_points_num)
                        {
                            has_ultra_obs = true;
                            break;
                        }
                    }
                }
            }

            bool is_sensor_error = (lidar_cnt == 1 && camera_cnt == 1 && ultra_cnt == 1);

            return is_sensor_error || has_ultra_obs || has_camera_obs || has_lidar_obs;
        }
    };
} // namespace CHARGE_ODT


#endif // OBSTACLE_DETECTION_H
