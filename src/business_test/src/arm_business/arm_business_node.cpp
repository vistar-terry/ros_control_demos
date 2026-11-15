/**
 * @file arm_business_node.cpp
 * @author 万俟淋曦 
 * @brief 机械臂随机点遍历节点入口，仅保留 main：初始化 ROS 后构造 ArmBusiness 并 run()。
 *        业务逻辑：/joint_states 读当前关节角 -> 正运动学求末端位姿 -> 在可达球内生成 8 个
 *        到肩距离互不相等且整条臂不触地的随机点 -> 数值逆运动学求各点关节角
 *        -> 通过 /arm_trajectory_controller/follow_joint_trajectory action 下发轨迹。
 * @date 2026-11-15 11:08:25
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "business_test/arm_business/arm_business.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_business");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    arm_business::ArmBusiness app(nh, private_nh);
    return app.run();
}
