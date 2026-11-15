/**
 * @file robot_kinematics.cpp
 * @author 万俟淋曦 
 * @brief robot_kinematics.h 中类的实现：URDF 运动链构建、正/逆运动学、肩关节位置、
 *        整条臂最低高度（地面碰撞检测），以及配套的刚体变换工具。
 * @date 2026-11-15 11:08:25
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "business_test/arm_business/robot_kinematics.h"

#include <algorithm>

#include <ros/ros.h>

namespace arm_business
{
namespace
{

/** 带部分主元的高斯消元，解 6x6 线性方程组 A * x = b */
bool solveLinear6(const double A[6][6], const double b[6], double x[6])
{
    double m[6][7];
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
            m[i][j] = A[i][j];
        m[i][6] = b[i];
    }

    for (int col = 0; col < 6; ++col)
    {
        int pivot = col;
        for (int r = col + 1; r < 6; ++r)
        {
            if (std::fabs(m[r][col]) > std::fabs(m[pivot][col]))
                pivot = r;
        }
        if (std::fabs(m[pivot][col]) < 1e-12)
            return false;
        if (pivot != col)
        {
            for (int c = col; c < 7; ++c)
                std::swap(m[pivot][c], m[col][c]);
        }
        for (int r = col + 1; r < 6; ++r)
        {
            const double factor = m[r][col] / m[col][col];
            for (int c = col; c < 7; ++c)
                m[r][c] -= factor * m[col][c];
        }
    }
    for (int i = 5; i >= 0; --i)
    {
        double sum = m[i][6];
        for (int j = i + 1; j < 6; ++j)
            sum -= m[i][j] * x[j];
        x[i] = sum / m[i][i];
    }
    return true;
}

} // namespace

// ===========================================================================
// TransformMath
// ===========================================================================

Transform TransformMath::identity()
{
    Transform t;
    for (int i = 0; i < 3; ++i)
    {
        t.p[i] = 0.0;
        for (int j = 0; j < 3; ++j)
            t.R[i][j] = (i == j) ? 1.0 : 0.0;
    }
    return t;
}

/** 复合两个变换：返回 a * b（先做 b，再做 a） */
Transform TransformMath::compose(const Transform &a, const Transform &b)
{
    Transform c;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k)
                sum += a.R[i][k] * b.R[k][j];
            c.R[i][j] = sum;
        }
        double sum = 0.0;
        for (int k = 0; k < 3; ++k)
            sum += a.R[i][k] * b.p[k];
        c.p[i] = a.p[i] + sum;
    }
    return c;
}

/** URDF 的 <origin xyz="..." rpy="..."> 解析结果 -> 齐次变换 */
Transform TransformMath::fromPose(const urdf::Pose &pose)
{
    Transform t = identity();
    const double x = pose.rotation.x;
    const double y = pose.rotation.y;
    const double z = pose.rotation.z;
    const double w = pose.rotation.w;

    t.R[0][0] = 1.0 - 2.0 * (y * y + z * z);
    t.R[0][1] = 2.0 * (x * y - w * z);
    t.R[0][2] = 2.0 * (x * z + w * y);
    t.R[1][0] = 2.0 * (x * y + w * z);
    t.R[1][1] = 1.0 - 2.0 * (x * x + z * z);
    t.R[1][2] = 2.0 * (y * z - w * x);
    t.R[2][0] = 2.0 * (x * z - w * y);
    t.R[2][1] = 2.0 * (y * z + w * x);
    t.R[2][2] = 1.0 - 2.0 * (x * x + y * y);

    t.p[0] = pose.position.x;
    t.p[1] = pose.position.y;
    t.p[2] = pose.position.z;
    return t;
}

/** 绕关节轴旋转 angle 弧度（Rodrigues 公式），axis 在关节局部坐标系中 */
Transform TransformMath::fromAxisAngle(const double axis[3], double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    Transform t = identity();
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            t.R[i][j] = (i == j ? c : 0.0) + (1.0 - c) * axis[i] * axis[j];
        }
    }
    t.R[0][1] -= s * axis[2];
    t.R[0][2] += s * axis[1];
    t.R[1][0] += s * axis[2];
    t.R[1][2] -= s * axis[0];
    t.R[2][0] -= s * axis[1];
    t.R[2][1] += s * axis[0];
    return t;
}

/** 旋转矩阵 -> 旋转向量（轴 * 角度），用于把姿态误差变成 3 维向量 */
void TransformMath::rotationVector(const double R[3][3], double v[3])
{
    double cos_theta = 0.5 * (R[0][0] + R[1][1] + R[2][2] - 1.0);
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    const double theta = std::acos(cos_theta);

    if (theta < 1e-6)
    {
        // 小角度下 rotvec ≈ 1/2 * vee(R - R^T)，避免除以 sin(theta)
        v[0] = 0.5 * (R[2][1] - R[1][2]);
        v[1] = 0.5 * (R[0][2] - R[2][0]);
        v[2] = 0.5 * (R[1][0] - R[0][1]);
        return;
    }
    if (theta > kPi - 1e-4)
    {
        // 接近 180°：用对角线求轴，用非对角元素定符号
        double axis[3];
        axis[0] = std::sqrt(std::max(0.0, 0.5 * (R[0][0] + 1.0)));
        axis[1] = std::sqrt(std::max(0.0, 0.5 * (R[1][1] + 1.0)));
        axis[2] = std::sqrt(std::max(0.0, 0.5 * (R[2][2] + 1.0)));
        if (R[2][1] - R[1][2] < 0.0)
            axis[0] = -axis[0];
        if (R[0][2] - R[2][0] < 0.0)
            axis[1] = -axis[1];
        if (R[1][0] - R[0][1] < 0.0)
            axis[2] = -axis[2];
        for (int i = 0; i < 3; ++i)
            v[i] = theta * axis[i];
        return;
    }
    const double k = theta / (2.0 * std::sin(theta));
    v[0] = k * (R[2][1] - R[1][2]);
    v[1] = k * (R[0][2] - R[2][0]);
    v[2] = k * (R[1][0] - R[0][1]);
}

// ===========================================================================
// RobotKinematics
// ===========================================================================

bool RobotKinematics::buildChain(const urdf::Model &model, const std::string &tip_link,
                                 std::vector<std::string> &joint_names, std::string &root_link)
{
    urdf::LinkConstSharedPtr link = model.getLink(tip_link);
    if (!link)
    {
        ROS_ERROR("Tip link [%s] not found in URDF", tip_link.c_str());
        return false;
    }

    // 从 tip 沿 parent_joint 回溯到 root，再反转成 base -> tip 的正向顺序
    std::vector<ChainNode> reversed;
    while (link && link->parent_joint)
    {
        const urdf::JointSharedPtr &joint = link->parent_joint;
        ChainNode node;
        node.fixed = TransformMath::fromPose(joint->parent_to_joint_origin_transform);
        node.axis[0] = node.axis[1] = node.axis[2] = 0.0;

        if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS)
        {
            node.joint_name = joint->name;
            const double ax = joint->axis.x;
            const double ay = joint->axis.y;
            const double az = joint->axis.z;
            const double norm = std::sqrt(ax * ax + ay * ay + az * az);
            if (norm < 1e-9)
            {
                ROS_ERROR("Joint [%s] has a zero-length rotation axis", joint->name.c_str());
                return false;
            }
            node.axis[0] = ax / norm;
            node.axis[1] = ay / norm;
            node.axis[2] = az / norm;

            // 读取 URDF 位置限位，供 IK 钳位使用：避免解超出硬件限位后被
            // 硬件接口 write() 钳位，导致实际位置与指令不符、轨迹跟踪超差而 ABORT。
            if (joint->type == urdf::Joint::CONTINUOUS || !joint->limits)
            {
                node.lower = -2.0 * kPi;
                node.upper = 2.0 * kPi;
            }
            else
            {
                node.lower = joint->limits->lower;
                node.upper = joint->limits->upper;
                if (node.lower > node.upper)
                {
                    node.lower = -2.0 * kPi;
                    node.upper = 2.0 * kPi;
                }
            }
        }
        else if (joint->type != urdf::Joint::FIXED)
        {
            ROS_ERROR("Unsupported type for joint [%s] (only revolute / continuous / fixed are supported)",
                      joint->name.c_str());
            return false;
        }

        reversed.push_back(node);
        link = model.getLink(joint->parent_link_name);
    }

    root_link = link ? link->name : std::string();
    chain_.assign(reversed.rbegin(), reversed.rend());
    for (size_t i = 0; i < chain_.size(); ++i)
    {
        if (!chain_[i].joint_name.empty())
            joint_names.push_back(chain_[i].joint_name);
    }
    return !chain_.empty() && !joint_names.empty();
}

Transform RobotKinematics::forwardKinematics(const std::vector<double> &q) const
{
    Transform t = TransformMath::identity();
    size_t k = 0;
    for (size_t i = 0; i < chain_.size(); ++i)
    {
        t = TransformMath::compose(t, chain_[i].fixed);
        if (!chain_[i].joint_name.empty() && k < q.size())
        {
            t = TransformMath::compose(t, TransformMath::fromAxisAngle(chain_[i].axis, q[k]));
            ++k;
        }
    }
    return t;
}

void RobotKinematics::shoulderPosition(double shoulder[3]) const
{
    Transform t = TransformMath::identity();
    for (size_t i = 0; i < chain_.size(); ++i)
    {
        t = TransformMath::compose(t, chain_[i].fixed);
        if (!chain_[i].joint_name.empty())
        {
            shoulder[0] = t.p[0];
            shoulder[1] = t.p[1];
            shoulder[2] = t.p[2];
            return;
        }
    }
    shoulder[0] = shoulder[1] = shoulder[2] = 0.0;
}

/**
 * 从第一个活动关节（肩）起，记录其后每一个连杆端点/关节原点/末端的 z：
 * 相邻端点之间是直杆连杆，z 沿杆线性变化，最小值必在端点处，故取端点最小值即可。
 * 肩之前的固定底座不计入（它本身立在地面上）。
 */
double RobotKinematics::armMinZ(const std::vector<double> &q) const
{
    Transform t = TransformMath::identity();
    double min_z = 1e9;
    size_t k = 0;
    bool started = false;
    for (size_t i = 0; i < chain_.size(); ++i)
    {
        t = TransformMath::compose(t, chain_[i].fixed);
        if (!chain_[i].joint_name.empty())
            started = true; // 第一个活动关节 = 肩，从这里起是运动臂
        if (started)
            min_z = std::min(min_z, t.p[2]);
        if (!chain_[i].joint_name.empty() && k < q.size())
        {
            t = TransformMath::compose(t, TransformMath::fromAxisAngle(chain_[i].axis, q[k]));
            ++k;
            if (started)
                min_z = std::min(min_z, t.p[2]);
        }
    }
    return (min_z > 1e8) ? t.p[2] : min_z;
}

/**
 * 把位姿写成 6 维向量：[x, y, z, rx, ry, rz]。
 * 旋转部分以 ref_R 为参考姿态，因此"保持姿态不变"等价于后 3 维保持为 0。
 * 用同一个函数做中心差分求雅可比，可以保证误差与雅可比严格自洽。
 */
void RobotKinematics::poseVector(const double ref_R[3][3], const std::vector<double> &q, double out[6]) const
{
    const Transform t = forwardKinematics(q);
    out[0] = t.p[0];
    out[1] = t.p[1];
    out[2] = t.p[2];

    // R_rel = ref_R^T * t.R
    double rel[3][3];
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rel[i][j] = ref_R[0][i] * t.R[0][j] +
                        ref_R[1][i] * t.R[1][j] +
                        ref_R[2][i] * t.R[2][j];
        }
    }
    TransformMath::rotationVector(rel, &out[3]);
}

/** 阻尼最小二乘数值逆解：dq = J^T (J J^T + lambda^2 I)^-1 e */
bool RobotKinematics::solveInverseKinematics(const double ref_R[3][3], const Transform &goal,
                                             std::vector<double> &q, double epsilon, int max_iterations,
                                             double damping, int &used_iterations, double &position_error) const
{
    const size_t n = q.size();
    const double delta = 1e-5; // 中心差分步长 (rad)
    used_iterations = 0;
    position_error = -1.0;

    // 把初值钳位到关节限位内，再记住它用于抑制零空间漂移
    size_t kc = 0;
    for (size_t i = 0; i < chain_.size() && kc < n; ++i)
    {
        if (chain_[i].joint_name.empty())
            continue;
        q[kc] = std::max(chain_[i].lower, std::min(q[kc], chain_[i].upper));
        ++kc;
    }
    const std::vector<double> q_pref = q;

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        used_iterations = iter + 1;
        double f[6];
        poseVector(ref_R, q, f);
        double e[6] = {goal.p[0] - f[0], goal.p[1] - f[1], goal.p[2] - f[2],
                       -f[3], -f[4], -f[5]};

        double err2 = 0.0;
        for (int r = 0; r < 6; ++r)
            err2 += e[r] * e[r];
        if (std::sqrt(err2) < epsilon)
        {
            const Transform t = forwardKinematics(q);
            position_error = std::sqrt((t.p[0] - goal.p[0]) * (t.p[0] - goal.p[0]) +
                                       (t.p[1] - goal.p[1]) * (t.p[1] - goal.p[1]) +
                                       (t.p[2] - goal.p[2]) * (t.p[2] - goal.p[2]));
            return true;
        }

        // 6 x n 雅可比，逐关节中心差分
        std::vector<std::vector<double>> J(6, std::vector<double>(n, 0.0));
        for (size_t i = 0; i < n; ++i)
        {
            double plus[6], minus[6];
            q[i] += delta;
            poseVector(ref_R, q, plus);
            q[i] -= 2.0 * delta;
            poseVector(ref_R, q, minus);
            q[i] += delta;
            for (int r = 0; r < 6; ++r)
                J[r][i] = (plus[r] - minus[r]) / (2.0 * delta);
        }

        double A[6][6];
        for (int r = 0; r < 6; ++r)
        {
            for (int c = 0; c < 6; ++c)
            {
                double sum = 0.0;
                for (size_t k = 0; k < n; ++k)
                    sum += J[r][k] * J[c][k];
                A[r][c] = sum + (r == c ? damping * damping : 0.0);
            }
        }

        double y[6];
        if (!solveLinear6(A, e, y))
            return false;

        std::vector<double> dq(n, 0.0);
        for (size_t i = 0; i < n; ++i)
        {
            for (int r = 0; r < 6; ++r)
                dq[i] += J[r][i] * y[r];
        }

        // 零空间偏好：把关节轻轻拉回初值，投影到雅可比零空间后再叠加，
        // 抑制迭代过程中关节在零空间里的无意义漂移（不影响末端位姿）。
        // 只在误差较大时生效：贴近目标后衰减为 0，避免干扰最终收敛精度。
        // 注：UR 默认上电位姿 wrist_2 = 0 属于腕部奇异位形，第一个目标点
        // 仍可能切换到另一组等价的手腕构型，之后各点保持稳定。
        const double pull = 0.05 * std::min(1.0, std::sqrt(err2) / 0.01);
        std::vector<double> bias(n, 0.0);
        double jb[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        for (size_t i = 0; i < n; ++i)
        {
            bias[i] = -pull * (q[i] - q_pref[i]);
            for (int r = 0; r < 6; ++r)
                jb[r] += J[r][i] * bias[i];
        }
        double yb[6];
        if (solveLinear6(A, jb, yb))
        {
            for (size_t i = 0; i < n; ++i)
            {
                for (int r = 0; r < 6; ++r)
                    bias[i] -= J[r][i] * yb[r];
                dq[i] += bias[i];
            }
        }

        double max_step = 0.0;
        for (size_t i = 0; i < n; ++i)
            max_step = std::max(max_step, std::fabs(dq[i]));
        // 限制单次迭代的最大关节增量，避免接近奇异位形时发散
        const double max_allowed = 0.25;
        const double scale = (max_step > max_allowed) ? max_allowed / max_step : 1.0;
        size_t k2 = 0;
        for (size_t i = 0; i < chain_.size() && k2 < n; ++i)
        {
            if (chain_[i].joint_name.empty())
                continue;
            q[k2] = std::max(chain_[i].lower, std::min(q[k2] + scale * dq[k2], chain_[i].upper));
            ++k2;
        }
    }
    return false;
}

} // namespace arm_business
