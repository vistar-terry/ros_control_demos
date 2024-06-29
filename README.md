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

![Peek 2024-06-29 16-36](img/Peek 2024-06-29 16-36.gif)



## 交流讨论



![git](img/git.png)

















