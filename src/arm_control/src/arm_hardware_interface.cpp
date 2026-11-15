/**
 * @file arm_hardware_interface.cpp
 * @author 万俟淋曦 
 * @brief UR20 机械臂硬件抽象层实现
 * @date 2026-11-15 11:08:25
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "arm_control/arm_hardware_interface.h"

#include <algorithm>
#include <cstddef>

namespace arm_control
{

namespace
{

// arm_description/urdf/ur20.xacro 生成的 6 个转动关节，前缀为空
const char* const kDefaultJoints[] = {
    "shoulder_pan_joint",    // 回转轴，连接底座，负责整个机械臂的水平旋转
    "shoulder_lift_joint",   // 下臂轴，控制机械臂前后摆动（肩部抬升），相当于肩关节
    "elbow_joint",           // 上臂轴，控制小臂的俯仰，相当于肘关节
    "wrist_1_joint",      // 腕部翻滚轴，相当于手腕旋转
    "wrist_2_joint",     // 腕部俯仰轴，相当于手腕上下摆动
    "wrist_3_joint"        // 腕部偏航轴，相当于手腕左右摆动
};

const std::size_t kDefaultJointCount = sizeof(kDefaultJoints) / sizeof(kDefaultJoints[0]);

}  // namespace

ArmHWInterface::ArmHWInterface(ros::NodeHandle& nh) : m_nh(nh)
{
}

bool ArmHWInterface::init()
{
    // 1. 关节列表：优先用参数服务器上的配置，缺失时回退到 UR20 默认列表
    std::vector<std::string> joint_names;
    if (!m_nh.getParam("hardware_interface/joints", joint_names) || joint_names.empty())
    {
        ROS_WARN("Param 'hardware_interface/joints' not found, using the 6 default UR20 joints.");
        joint_names.assign(kDefaultJoints, kDefaultJoints + kDefaultJointCount);
    }

    // 一次性填充满 m_joints：注册句柄之后不能再发生扩容，否则句柄里的指针会失效
    m_joints.clear();
    m_joints.reserve(joint_names.size());
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        m_joints.push_back(JointInfo(joint_names[i]));
    }

    // 2. 初始位姿：让机械臂上电时停在一个"折起来"的姿态，便于 RViz 观察
    std::vector<double> initial_positions;
    if (m_nh.getParam("hardware_interface/initial_positions", initial_positions))
    {
        if (initial_positions.size() == m_joints.size())
        {
            for (std::size_t i = 0; i < m_joints.size(); ++i)
            {
                m_joints[i].pos = initial_positions[i];
                m_joints[i].cmd = initial_positions[i];
            }
        }
        else
        {
            ROS_WARN("'initial_positions' has %zu values but %zu joints, ignored.",
                     initial_positions.size(), m_joints.size());
        }
    }

    // 3. 关节限位：与控制器共用同一份 'joint_limits' 参数，write() 里做钳位
    for (std::size_t i = 0; i < m_joints.size(); ++i)
    {
        JointInfo &joint = m_joints[i];
        const std::string base = "joint_limits/" + joint.name;
        m_nh.getParam(base + "/min_position", joint.min_pos);
        m_nh.getParam(base + "/max_position", joint.max_pos);

        if (joint.min_pos > joint.max_pos)
        {
            ROS_ERROR("Invalid position limits for joint '%s': min > max", joint.name.c_str());
            return false;
        }
        joint.pos = std::max(joint.min_pos, std::min(joint.pos, joint.max_pos));
        joint.cmd = joint.pos;

        ROS_INFO("arm joint [%zu]: %s, start = %.3f rad, limits = [%.3f, %.3f]",
                 i, joint.name.c_str(), joint.pos, joint.min_pos, joint.max_pos);
    }

    // 4. 注册 ros_control 接口
    //    每个关节既要能被读状态，也要能接受位置指令
    for (std::size_t i = 0; i < m_joints.size(); ++i)
    {
        JointInfo &joint = m_joints[i];

        hardware_interface::JointStateHandle state_handle(
            joint.name, &joint.pos, &joint.vel, &joint.eff);
        m_jnt_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(
            m_jnt_state_interface.getHandle(joint.name), &joint.cmd);
        m_jnt_pos_interface.registerHandle(pos_handle);
    }

    registerInterface(&m_jnt_state_interface);
    registerInterface(&m_jnt_pos_interface);

    ROS_INFO("Arm hardware interface initialized with %zu joints.", m_joints.size());
    return true;
}

void ArmHWInterface::read(const ros::Duration & /*period*/)
{
    // 接真实机器人时，在这里读编码器/驱动器，刷新 pos、vel、eff。
    // 本示例没有物理硬件：仿真状态由 write() 同步推进，所以此处无需操作。
    // 控制循环顺序为 read -> cm.update -> write，控制器读到的是上一周期的状态，
    // 这与真实系统存在一拍采样延迟的情况一致。
}

void ArmHWInterface::write(const ros::Duration &period)
{
    const double dt = period.toSec();

    for (std::size_t i = 0; i < m_joints.size(); ++i)
    {
        JointInfo &joint = m_joints[i];

        // 位置限幅：模拟驱动器对越界指令的保护，同时把钳位结果写回 cmd，
        // 保证控制器看到的指令值与反馈值一致。
        const double target = std::max(joint.min_pos, std::min(joint.cmd, joint.max_pos));
        joint.cmd = target;

        // 理想位置执行器：指令即反馈。vel 由位置差分得到，无负载时 eff 为 0。
        // 若想让运动更平滑，可以在这里把 (target - pos) 限制在 max_velocity*dt 内。
        if (dt > 0.0)
        {
            joint.vel = (target - joint.pos) / dt;
        }
        joint.pos = target;
        joint.eff = 0.0;
    }
}

}  // namespace arm_control
