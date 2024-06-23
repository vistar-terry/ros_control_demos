#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist moveRobot(const double& linear, const double& angular)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear; // 线速度
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = angular; // 角速度
    ROS_INFO("moveRobot, linear: %.3lf, angular: %.1lf", linear, angular*180/M_PI);

    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diff_drive_business");
    ros::NodeHandle nh;
    ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("/diff_drive_controller/cmd_vel", 10);

    ros::Rate rate(10);

    while (ros::ok())
    {
        velPub.publish(moveRobot(0.5, 0));
        ros::Duration(3.0).sleep();
        velPub.publish(moveRobot(0, 1.57));
        ros::Duration(1.0).sleep();

        rate.sleep();
    }

    return 0;
}
