
/**
 * @file robot_kinematics.cpp
 * @author 万俟淋曦 
 * @brief 运动学类声明：URDF 运动链构建、正/逆运动学、肩关节位置、
 *        整条臂最低高度（地面碰撞检测），以及配套的刚体变换工具。
 * @date 2026-11-15 11:08:25
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef BUSINESS_TEST_ROBOT_KINEMATICS_H
#define BUSINESS_TEST_ROBOT_KINEMATICS_H

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <urdf/model.h>

namespace arm_business
{

const double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// 极简刚体变换：R 为 3x3 旋转矩阵（列向量约定：第 j 列 = 第 j 轴），p 为平移
// ---------------------------------------------------------------------------
struct Transform
{
    double R[3][3];
    double p[3];
};

/** 变换工具（纯静态函数） */
class TransformMath
{
public:
    static Transform identity();
    static Transform compose(const Transform &a, const Transform &b);
    static Transform fromPose(const urdf::Pose &pose);
    static Transform fromAxisAngle(const double axis[3], double angle);
    static void rotationVector(const double R[3][3], double v[3]);
};

// ---------------------------------------------------------------------------
// 运动链节点
// ---------------------------------------------------------------------------
struct ChainNode
{
    std::string joint_name; // 活动关节名；固定关节为空
    Transform fixed;        // 父连杆 -> 关节坐标系的固定变换
    double axis[3];         // 活动关节的旋转轴（关节局部系）
    double lower;           // 活动关节位置下限 (rad)，固定关节忽略
    double upper;           // 活动关节位置上限 (rad)，固定关节忽略
};

typedef std::vector<ChainNode> KinematicChain;

/**
 * @class RobotKinematics
 * @brief 从 URDF 构建 root -> tip 运动链，并自包含实现正/逆运动学。
 *        不依赖 MoveIt / KDL / Eigen。
 */
class RobotKinematics
{
public:
    RobotKinematics() {}
    ~RobotKinematics() {}

    /** 从 URDF 的 tip_link 回溯构建运动链；成功时填充 joint_names 与 root_link */
    bool buildChain(const urdf::Model &model, const std::string &tip_link,
                    std::vector<std::string> &joint_names, std::string &root_link);

    /** 正运动学：q 顺序与 chain 中活动关节顺序一致 */
    Transform forwardKinematics(const std::vector<double> &q) const;

    /** 第一个活动关节（肩）在根连杆系下的位置 */
    void shoulderPosition(double shoulder[3]) const;

    /** 给定关节角下"运动臂"的最低高度（根连杆系 z），用于地面碰撞检测 */
    double armMinZ(const std::vector<double> &q) const;

    /** 阻尼最小二乘数值逆解；q 既是初值也是输出 */
    bool solveInverseKinematics(const double ref_R[3][3], const Transform &goal,
                                std::vector<double> &q, double epsilon, int max_iterations,
                                double damping, int &used_iterations, double &position_error) const;

    bool empty() const { return chain_.empty(); }

private:
    /** 把位姿写成 6 维向量：[x, y, z, rx, ry, rz]，旋转以 ref_R 为参考 */
    void poseVector(const double ref_R[3][3], const std::vector<double> &q, double out[6]) const;

    KinematicChain chain_;
};

} // namespace arm_business

#endif // BUSINESS_TEST_ROBOT_KINEMATICS_H
