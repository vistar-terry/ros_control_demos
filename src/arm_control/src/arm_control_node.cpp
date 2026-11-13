/**
 * @file arm_control_node.cpp
 * @brief 机械臂 ros_control 节点：驱动硬件接口的实时控制循环
 *
 * 循环顺序：read() -> controller_manager.update() -> write()
 * controller_manager 会按配置加载/启动控制器，并把控制指令写进
 * ArmHWInterface 注册的 PositionJointInterface。
 */
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "arm_control/arm_hardware_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_control_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double control_rate = 100.0;  // Hz
  private_nh.param("control_rate", control_rate, 100.0);
  if (control_rate <= 0.0)
  {
    ROS_WARN("control_rate must be positive, falling back to 100 Hz.");
    control_rate = 100.0;
  }

  arm_control::ArmHWInterface arm(nh);
  if (!arm.init())
  {
    ROS_FATAL("Failed to initialize the arm hardware interface.");
    return 1;
  }

  controller_manager::ControllerManager cm(&arm, nh);

  // 轨迹控制器的 action server、controller_manager 的 service 都需要回调线程
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate rate(control_rate);
  ros::Time last_time = ros::Time::now();

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    ros::Duration period = now - last_time;
    last_time = now;

    // 首帧或使用仿真时间时 period 可能为 0，此时退回名义周期
    if (period.toSec() <= 0.0 || period.toSec() > 1.0)
    {
      period = rate.expectedCycleTime();
    }

    arm.read(period);
    cm.update(now, period);
    arm.write(period);

    rate.sleep();
  }

  spinner.stop();
  return 0;
}
