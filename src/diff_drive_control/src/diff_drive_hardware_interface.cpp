#include "diff_drive_control/diff_drive_hardware_interface.h"

DiffDriveHWInterface::DiffDriveHWInterface(ros::NodeHandle &nh) : m_nh(nh)
{
}

/**
 * @brief 初始化关节信息
 *        注册抽象硬件接口
 * 
 */
void DiffDriveHWInterface::init()
{
    std::vector<std::string> joint_names;
    m_nh.getParam("/hardware_interface/joints", joint_names);
    for (std::string name : joint_names)
    {
        m_joints.push_back(JointInfo(name));
    }

    for (auto &joint : m_joints)
    {
        ROS_INFO("get joint: %s", joint.name.c_str());

        // Initialize hardware interface
        hardware_interface::JointStateHandle state_handle(joint.name, &joint.pos, &joint.vel, &joint.eff);
        m_jnt_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle vel_handle(m_jnt_state_interface.getHandle(joint.name), &joint.cmd);
        m_jnt_vel_interface.registerHandle(vel_handle);
    }

    registerInterface(&m_jnt_state_interface);
    registerInterface(&m_jnt_vel_interface);
}

void DiffDriveHWInterface::read(const ros::Duration &period)
{
    // Read the state of the hardware (e.g., from sensors)
}

void DiffDriveHWInterface::write(const ros::Duration &period)
{
    // Send the command to the hardware (e.g., to actuators)
    for (auto &joint : m_joints)
    {
        joint.pos += joint.vel * period.toSec();
        // if (joint.vel != joint.cmd)
        // {
        //     ROS_INFO("write, joint: %s, cmd: %lf", joint.name.c_str(), joint.cmd);
        // }
        joint.vel = joint.cmd;
    }
}
