/**
 * @file diff_drive_hardware_interface.h
 * @author 万俟淋曦 
 * @brief 两轮差速抽象硬件接口
 * @date 2024-06-28 11:03:47
 * 
 * @copyright Copyright (c) 2024
 * 
 */
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

    // 关节信息结构
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
    // 硬件写接口
    void read(const ros::Duration &period);
    // 硬件读接口
    void write(const ros::Duration &period);

private:
    ros::NodeHandle m_nh;
    // 关节状态读取接口
    hardware_interface::JointStateInterface m_jnt_state_interface;
    // 关节速度控制接口
    hardware_interface::VelocityJointInterface m_jnt_vel_interface;
    // 关节信息存储向量
    std::vector<JointInfo> m_joints;
};

#endif // DIFF_DRIVE_HARDWARE_INTERFACE_H
