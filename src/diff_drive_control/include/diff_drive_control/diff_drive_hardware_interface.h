#ifndef DIFF_DRIVE_HARDWARE_INTERFACE_H
#define DIFF_DRIVE_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class DiffDriveHWInterface : public hardware_interface::RobotHW
{
public:
    DiffDriveHWInterface(ros::NodeHandle &nh);
    void init();
    void read(const ros::Duration &period);
    void write(const ros::Duration &period);

private:
    ros::NodeHandle m_nh;
    hardware_interface::JointStateInterface m_jnt_state_interface;
    hardware_interface::VelocityJointInterface m_jnt_vel_interface;
    std::vector<std::string> m_joint_names;
    std::vector<double> m_cmd;
    std::vector<double> m_pos;
    std::vector<double> m_vel;
    std::vector<double> m_eff;
};

#endif // DIFF_DRIVE_HARDWARE_INTERFACE_H
