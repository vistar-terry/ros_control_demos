/**
 * @file arm_hardware_interface.h
 * @brief UR20 机械臂的 ros_control 硬件抽象层
 *
 * URDF (urdf/inc/ur_transmissions.xacro) 中 6 个关节的 transmission 声明为
 * hardware_interface/PositionJointInterface，因此这里注册状态接口 +
 * 位置指令接口，供 joint_state_controller / joint_trajectory_controller 使用。
 *
 * 本示例没有真实驱动器：write() 中用一个"理想位置执行器"模拟被控对象，
 * 把控制器下发的位置指令直接当作编码器反馈。接真实机器人时，只需要把
 * read() 换成读编码器、write() 换成下发总线指令，控制器侧无需改动。
 */
#ifndef ARM_CONTROL_ARM_HARDWARE_INTERFACE_H
#define ARM_CONTROL_ARM_HARDWARE_INTERFACE_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace arm_control
{

class ArmHWInterface : public hardware_interface::RobotHW
{
public:
  /** 单个关节的全部状态与指令，接口句柄直接持有这些成员的地址 */
  struct JointInfo
  {
    std::string name;   // 关节名，必须与 URDF / 配置中的名字一致
    double cmd;         // 控制器写入的位置指令 (rad)
    double pos;         // 反馈位置 (rad)
    double vel;         // 反馈速度 (rad/s)
    double eff;         // 反馈力矩 (N*m)
    double min_pos;     // 位置下限 (rad)
    double max_pos;     // 位置上限 (rad)

    JointInfo()
      : name(""), cmd(0.0), pos(0.0), vel(0.0), eff(0.0),
        min_pos(-6.283185307179586), max_pos(6.283185307179586)
    {}

    explicit JointInfo(const std::string& name_)
      : name(name_), cmd(0.0), pos(0.0), vel(0.0), eff(0.0),
        min_pos(-6.283185307179586), max_pos(6.283185307179586)
    {}
  };

public:
  explicit ArmHWInterface(ros::NodeHandle& nh);

  /** 读取关节列表/初始位姿/限位，并注册硬件接口；失败返回 false */
  bool init();

  /** 从硬件读取状态（本示例为仿真，状态在 write() 中更新） */
  void read(const ros::Duration& period);

  /** 把控制器的指令下发给硬件（本示例同时推进仿真状态） */
  void write(const ros::Duration& period);

private:
  ros::NodeHandle m_nh;

  hardware_interface::JointStateInterface   m_jnt_state_interface;
  hardware_interface::PositionJointInterface m_jnt_pos_interface;

  // 注意：注册句柄时保存了 JointInfo 成员的地址，此后不得再改变 vector 容量
  std::vector<JointInfo> m_joints;
};

}  // namespace arm_control

#endif  // ARM_CONTROL_ARM_HARDWARE_INTERFACE_H
