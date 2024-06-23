#include "diff_drive_control/diff_drive_hardware_interface.h"

DiffDriveHWInterface::DiffDriveHWInterface(ros::NodeHandle &nh) : m_nh(nh)
{
}

void DiffDriveHWInterface::init()
{
    m_cmd = {0.0, 0.0, 0.0, 0.0};
    m_pos = {0.0, 0.0, 0.0, 0.0};
    m_vel = {0.0, 0.0, 0.0, 0.0};
    m_eff = {0.0, 0.0, 0.0, 0.0};

    m_nh.getParam("/hardware_interface/joints", m_joint_names);

    for (size_t i = 0; i < m_joint_names.size(); i++)
    {
        ROS_INFO("get joint: %s", m_joint_names[i].c_str());

        // Initialize hardware interface
        hardware_interface::JointStateHandle state_handle(m_joint_names[i], &m_pos[i], &m_vel[i], &m_eff[i]);
        m_jnt_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle vel_handle(m_jnt_state_interface.getHandle(m_joint_names[i]), &m_cmd[i]);
        m_jnt_vel_interface.registerHandle(vel_handle);
    }

    registerInterface(&m_jnt_state_interface);
    registerInterface(&m_jnt_vel_interface);
}

void DiffDriveHWInterface::read(const ros::Duration &period)
{
    // Read the state of the hardware (e.g., from sensors)
    for (size_t i = 0; i < m_joint_names.size(); i++)
    {
        m_pos[i] += m_vel[i] * period.toSec();
    }
}

void DiffDriveHWInterface::write(const ros::Duration &period)
{
    // Send the command to the hardware (e.g., to actuators)
    for (size_t i = 0; i < m_joint_names.size(); i++)
    {
        if (m_vel[i] != m_cmd[i])
        {
            ROS_INFO("write, m_cmd: %lf", m_cmd[i]);
        }
        m_vel[i] = m_cmd[i];
    }
}
