#ifndef TRAJECTORY_BUILDER_H_
#define TRAJECTORY_BUILDER_H_

// main ROS header
#include <ros/ros.h>

// generically libraries
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>

// ROS messages
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>


// class definition
class TrajectoryBuilder{

private:
    //constants and parameters: changeable via public "set" functions
    double speed_max_;
    double omega_max_;
    double accel_max_;
    double alpha_max_;

    double dt_;
    double path_move_tol_;

    //member vars of class
    geometry_msgs::Twist halt_twist_;

public:

    TrajectoryBuilder();    // constructor
    ~TrajectoryBuilder();   // destructor

    // setters functions
    void set_speed_max(double speed) {
        speed_max_ = speed;
        ROS_INFO("speed_max set to: %f", speed);
    }

    void set_omega_max(double omega) {
        omega_max_ = omega;
        ROS_INFO("omega_max set to: %f", omega);
    }

    void set_accel_max(double accel) {
        accel_max_ = accel;
        ROS_INFO("accel_max set to: %f", accel);
    }

    void set_alpha_max(double alpha) {
        alpha_max_ = alpha;
        ROS_INFO("alpha_max set to: %f", alpha);
    }

    void set_dt(double dt) {
        dt_ = dt;
        ROS_INFO("dt set to: %f", dt);
    }

    void set_path_move_tol_(double tol) {
        path_move_tol_ = tol;
        ROS_INFO("path_move_tol set to: %f", tol);
    }


    // useful functions:
    double min_dang(double dang);
    double sat(double x);
    double sgn(double x);
    double convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi);

    // fill a PoseStamped object from planar x,y,phi info
    geometry_msgs::PoseStamped xyPsi2PoseStamped(double x, double y, double psi);

    // main traj-builder functions:
    void build_trapezoidal_spin_traj( geometry_msgs::PoseStamped start_pose,
                                      geometry_msgs::PoseStamped end_pose,
                                      std::vector<nav_msgs::Odometry> &vec_of_states );

    void build_triangular_spin_traj( geometry_msgs::PoseStamped start_pose,
                                     geometry_msgs::PoseStamped end_pose,
                                     std::vector<nav_msgs::Odometry> &vec_of_states );

    void build_spin_traj( geometry_msgs::PoseStamped start_pose,
                          geometry_msgs::PoseStamped end_pose,
                          std::vector<nav_msgs::Odometry> &vec_of_states );

    void build_trapezoidal_travel_traj( geometry_msgs::PoseStamped start_pose,
                                        geometry_msgs::PoseStamped end_pose,
                                        std::vector<nav_msgs::Odometry> &vec_of_states );

    void build_triangular_travel_traj( geometry_msgs::PoseStamped start_pose,
                                       geometry_msgs::PoseStamped end_pose,
                                       std::vector<nav_msgs::Odometry> &vec_of_states );

    void build_travel_traj( geometry_msgs::PoseStamped start_pose,
                            geometry_msgs::PoseStamped end_pose,
                            std::vector<nav_msgs::Odometry> &vec_of_states );

    void build_point_and_go_traj( geometry_msgs::PoseStamped start_pose,
                                  geometry_msgs::PoseStamped end_pose,
                                  std::vector<nav_msgs::Odometry> &vec_of_states );

};  // end of class definition

#endif
