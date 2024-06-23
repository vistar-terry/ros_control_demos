#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

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

void robotVelCallback(const geometry_msgs::TwistStamped& msg)
{
    static double lastVel = 0.0;
    static double lastAng = 0.0;
    double curVel = msg.twist.linear.x;
    double curAng = msg.twist.angular.z;
    ROS_INFO("robotVelCallback, linear: %.3lf, angular: %.1lf", curVel, curAng*180/M_PI);
    if (lastVel != curVel || lastAng != curAng)
    {
        ROS_INFO("robotVelCallback, linear: %.3lf, angular: %.1lf", curVel, curAng*180/M_PI);
        lastVel = curVel;
        lastAng = curAng;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diff_drive_business");
    ros::NodeHandle nh;

    ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("/diff_drive_controller/cmd_vel", 10);
    ros::Subscriber velSub = nh.subscribe("/diff_drive_controller/cmd_vel_out", 10, robotVelCallback);

    ros::Rate rate(10);

    while (ros::ok())
    {
        velPub.publish(moveRobot(0.5, -0.3));
        ros::Duration(3.0).sleep();
        velPub.publish(moveRobot(-0.5, -0.3));
        ros::Duration(3.0).sleep();

        rate.sleep();
    }

    return 0;
}
