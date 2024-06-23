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
    struct JointInfo
    {
        std::string name;
        double cmd;
        double pos;
        double vel;
        double eff;

        JointInfo() : name(""), cmd(0.0), pos(0.0), vel(0.0), eff(0.0)
        {}

        JointInfo(std::string name_) 
            : name(name_), cmd(0.0), pos(0.0), vel(0.0), eff(0.0)
        {}

        JointInfo(std::string name_, double cmd_, double pos_, double vel_, double dff_) 
            : name(name_), cmd(cmd_), pos(pos_), vel(vel_), eff(dff_)
        {}

    };
    
public:
    DiffDriveHWInterface(ros::NodeHandle &nh);
    void init();
    void read(const ros::Duration &period);
    void write(const ros::Duration &period);

private:
    ros::NodeHandle m_nh;
    hardware_interface::JointStateInterface m_jnt_state_interface;
    hardware_interface::VelocityJointInterface m_jnt_vel_interface;
    std::vector<JointInfo> m_joints;
};

#endif // DIFF_DRIVE_HARDWARE_INTERFACE_H
