/**
 * @file arm_business.cpp
 * @author 万俟淋曦 
 * @brief 业务主类声明：读取参数 -> 加载 URDF -> 生成 8 个不触地的可达球随机点并逆解
 *        -> 发布 RViz 标记 -> 通过 action 循环下发轨迹。
 * @date 2026-11-15 11:08:25
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef BUSINESS_TEST_ARM_BUSINESS_H
#define BUSINESS_TEST_ARM_BUSINESS_H

#include <cstddef>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/topic.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <visualization_msgs/MarkerArray.h>

#include "business_test/arm_business/robot_kinematics.h"

namespace arm_business
{

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

// ---------------------------------------------------------------------------
// 轴对齐包围盒（工作空间可视化用）
// ---------------------------------------------------------------------------
struct BoundingBox
{
    double lo[3];
    double hi[3];
};

/**
 * @class ArmBusiness
 * @brief 业务主类：封装参数、ROS 通信与完整的随机点遍历流程。
 */
class ArmBusiness
{
public:
    ArmBusiness(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~ArmBusiness() {}

    /** 运行完整业务流程，返回进程退出码 */
    int run();

private:
    // 目标点 / 逆解生成
    bool generateTargets();
    // 姿态参考矩阵：末端 Z 轴平行于肩->点半径，正向指向球外
    void radialToolRotation(const double p[3], const double shoulder[3],
                            const double startup_R[3][3], double roll, double R[3][3]) const;
    // 在可达球内生成 count 个到球心距离互不相等的随机点
    void randomSpherePoints(const double shoulder[3], double max_radius, double min_z,
                            int count, std::mt19937 &rng, const Transform &pose,
                            std::vector<Transform> &out) const;
    // 工作空间包围盒与 RViz 标记
    BoundingBox computeWorkspaceBox(const double shoulder[3], double reach,
                                    double margin, double floor_z) const;
    visualization_msgs::MarkerArray makeWorkspaceMarkers(const std::vector<Transform> &targets,
                                                         const std::string &frame_id) const;

    ros::NodeHandle &nh_;
    ros::NodeHandle &pnh_;

    RobotKinematics kinematics_;
    std::vector<std::string> joint_names_;
    std::string root_link_;
    std::size_t dof_;

    ros::Publisher marker_pub_;
    std::unique_ptr<TrajectoryClient> client_;

    // 参数
    std::string action_name_;
    std::string joint_states_topic_;
    std::string description_param_;
    std::string marker_topic_;
    std::string tip_link_;
    double cube_side_;
    double bbox_margin_;
    double max_reach_;
    double reach_ratio_;
    double min_z_;
    double link_radius_;
    double ground_clearance_;
    double traverse_time_;
    double hold_time_;
    double ik_epsilon_;
    double ik_damping_;
    int loops_;
    int ik_max_iterations_;
    int orientation_mode_;
    double tool_roll_;
    bool random_order_;

    // 运行态
    std::vector<double> q_;        // 启动关节角（逆解初值）
    Transform center_;             // 启动末端位姿
    double ref_R_[3][3];           // 启动姿态参考矩阵
    double shoulder_[3];           // 肩关节位置
    std::vector<Transform> targets_;
    std::vector<std::vector<double>> q_targets_;
};

} // namespace arm_business

#endif // BUSINESS_TEST_ARM_BUSINESS_H
