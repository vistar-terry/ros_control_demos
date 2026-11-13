# ros_control_demos

ros_control 的简单示例，新手友好

本项目链接：[https://github.com/vistar-terry/ros_control_demos](https://github.com/vistar-terry/ros_control_demos)

有不对的地方或其他疑问，欢迎提 [issues](https://github.com/vistar-terry/ros_control_demos/issues)。



## 目录说明

1. arm_control：机械臂 ros_control 硬件抽象
2. arm_description：机械臂模型描述
3. business_test：模拟业务控制
4. diff_drive_control：两轮差速机器人 ros_control 硬件抽象
5. diff_drive_description：两轮差速机器人模型描述



## 安装方法

1. 下载

    ```bash
    git clone https://github.com/vistar-terry/ros_control_demos.git
    ```

2. 编译

    ```bash
    cd ros_control_demos
    catkin_make
    ```

3. 添加环境变量

    ```bash
    source devel/setup.bash
    ```

4. 运行两轮差速机器人 ros_control 节点，并控制机器人移动

    ```bash
    roslaunch business_test diff_drive_business.launch
    ```

​		在rviz中的可视化结果：

![Peek 2024-06-29 16-36](img/Peek_2024-06-29_16-36.gif)

5. 运行机械臂 ros_control 节点

    ```bash
    # 默认启动位置轨迹控制器，提供 follow_joint_trajectory action（可对接 MoveIt）
    roslaunch arm_control arm_control.launch

    # 或者改用简单的一组关节角度控制器
    roslaunch arm_control arm_control.launch controller:=arm_position_controller
    rostopic pub -1 /arm_position_controller/command std_msgs/Float64MultiArray \
      "data: [0.0, -1.57, 1.57, -1.57, 0.0, 0.0]"
    ```

    `arm_control` 包含 UR20 的 6 自由度硬件抽象（`PositionJointInterface`）、
    `arm_control_node` 的 read → update → write 控制循环，以及关节限位、
    `joint_state_controller`、轨迹控制器和组位置控制器的配置。
    轨迹模式下可以用 `rqt_joint_trajectory_controller` 拖动关节，
    也可以向 `/arm_trajectory_controller/follow_joint_trajectory` 发送 action 目标；
    最简单的是直接用话题接口发一段轨迹（无需额外客户端）：

    ```bash
    rostopic pub -1 /arm_trajectory_controller/command trajectory_msgs/JointTrajectory \
      "{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0.5, -1.2, 1.2, -1.0, 0.0, 0.0], time_from_start: {secs: 3, nsecs: 0}}]}"
    ```

    加 `rviz:=false` 可关闭可视化窗口。

    依赖：`rosdep install --from-paths src --ignore-src -r -y`（需要
    `joint_trajectory_controller`、`position_controllers` 等 ros_controllers 包）。


## 交流讨论



![git](img/git.png)

















