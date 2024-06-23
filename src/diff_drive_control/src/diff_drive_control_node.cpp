#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "diff_drive_control/diff_drive_hardware_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_drive_control_node");
    ros::NodeHandle nh;

    DiffDriveHWInterface diff_drive(nh);
    diff_drive.init();

    controller_manager::ControllerManager cm(&diff_drive, nh);

    ros::Rate rate(50.0);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        ros::Duration period = rate.expectedCycleTime();
        // diff_drive.read(period);
        
        diff_drive.write(period);
        cm.update(ros::Time::now(), period);
        rate.sleep();
    }

    return 0;
}
