#ifndef TURTLEBOT_LINEAR_CONTROL_H_
#define TURTLEBOT_LINEAR_CONTROL_H_

// Main ROS header
#include <ros/ros.h>

// General purpose libraries
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// ROS messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>


// Global variables
const double UPDATE_RATE = 50.0;        // publication rate

// control gains
const double K_PSI = 10.0;
const double K_DISP = 3.0;
const double K_TRIP_DIST = 1.0;

// actuator limits
const double MAX_SPEED = 0.13;           // between [0, 1]
const double MAX_OMEGA = 0.91;           // between [0, 1]


// class definition
class LinearController{

private:
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber des_states_sub_;

    // Publishers
    ros::Publisher vel_pub_;
    ros::Publisher vel_pub2_;
    ros::Publisher errs_pub_;
    ros::Publisher state_errors_pub_;

    // Message definition
    geometry_msgs::Twist vel_cmd_;
    geometry_msgs::TwistStamped vel_cmd2_;
    double current_speed_des_;
    double current_omega_des_;

    // State values from topic /odom; these will get filled in by odomCallback()
    nav_msgs::Odometry current_odom_;
    double odom_vel_;
    double odom_omega_;
    double odom_x_;
    double odom_y_;
    double odom_psi_;
    geometry_msgs::Quaternion odom_quat_;

    // State values from topic /desired_states; these will get filled in by desStatesCallback();
    nav_msgs::Odometry des_states_;
    double des_states_vel_;
    double des_states_omega_;
    double des_states_x_;
    double des_states_y_;
    double des_states_psi_;
    geometry_msgs::Quaternion des_states_quat_;

    // message to publish state errors
    std_msgs::Float32MultiArray errs_;
    geometry_msgs::PointStamped state_errors_;


    // Member methods
    void initializeSubscribers();
    void initializePublishers();

    // Callback functions
    void odomCallback(const nav_msgs::Odometry &odom_msg);
    void desStatesCallback(const nav_msgs::Odometry &des_states_msg);


public:

    LinearController(ros::NodeHandle &nh);
    ~LinearController();

    void control_algorithm();
    double convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion);
    double min_dang(double dang);
    double sat(double x);

};  // end of class definition

#endif
