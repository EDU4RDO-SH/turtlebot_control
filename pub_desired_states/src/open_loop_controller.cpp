#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// node to receive desired states and republish the twist as cmd_vel commands
ros::Publisher g_twist_republisher;

// simply copy the desired twist and republish it to cmd_vel
void desiredStatesCallback(const nav_msgs::Odometry &des_state){
    geometry_msgs::Twist twist = des_state.twist.twist;
    g_twist_republisher.publish(twist);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "open_loop_controller_node");
    ros::NodeHandle nh; // two lines to create a publisher object that can talk to ROS

    ros::Subscriber des_state_subscriber = nh.subscribe("/desired_states", 1, desiredStatesCallback);
    g_twist_republisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ROS_INFO("Ready to republish velocity commands");

    ros::spin();

    return 0;
}
