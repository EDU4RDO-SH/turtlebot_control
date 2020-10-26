#include<trajectory_builder.h>

// constructor: fill in default param values (changeable via "set" fncs) -------
TrajectoryBuilder::TrajectoryBuilder(){

    //define a halt state;
    halt_twist_.linear.x = 0.0;
    halt_twist_.linear.y = 0.0;
    halt_twist_.linear.z = 0.0;
    halt_twist_.angular.x = 0.0;
    halt_twist_.angular.y = 0.0;
    halt_twist_.angular.z = 0.0;
}


// destructor ------------------------------------------------------------------
TrajectoryBuilder::~TrajectoryBuilder(){

}


// function to choose shortest angular distance, considering periodicity -------
double TrajectoryBuilder::min_dang(double dang) {
    while (dang > M_PI) dang -= 2.0 * M_PI;
    while (dang < -M_PI) dang += 2.0 * M_PI;
    return dang;
}


// saturation function, limits values to range -1 to 1 -------------------------
double TrajectoryBuilder::sat(double x) {
    if (x > 1.0) {
        return 1.0;
    }
    if (x< -1.0) {
        return -1.0;
    }
    return x;
}


// sign function, returns the sign of a value ----------------------------------
double TrajectoryBuilder::sgn(double x) {
    if (x > 0.0) {
        return 1.0;
    }
    if (x< 0.0) {
        return -1.0;
    }
    return 0.0;
}


// converts a quaternion to a scalar heading -----------------------------------
double TrajectoryBuilder::convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double psi = 2.0 * atan2(quat_z, quat_w);
    return psi;
}


// convert a heading scalar to a quaternion ------------------------------------
geometry_msgs::Quaternion TrajectoryBuilder::convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}


// fill a PoseStamped object from planar x,y,psi info --------------------------
geometry_msgs::PoseStamped TrajectoryBuilder::xyPsi2PoseStamped(double x, double y, double psi) {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose.orientation = convertPlanarPsi2Quaternion(psi);
    poseStamped.pose.position.x = x;
    poseStamped.pose.position.y = y;
    poseStamped.pose.position.z = 0.0;
    return poseStamped;
}


// TRAPEZOIDAL SPIN TRAJECTORY -------------------------------------------------
void TrajectoryBuilder::build_trapezoidal_spin_traj( geometry_msgs::PoseStamped start_pose,
                                               geometry_msgs::PoseStamped end_pose,
                                               std::vector<nav_msgs::Odometry> &vec_of_states ){
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
    double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    double t_ramp = omega_max_/ alpha_max_;
    double ramp_up_dist = 0.5 * alpha_max_ * t_ramp*t_ramp;
    double cruise_distance = fabs(dpsi) - 2.0 * ramp_up_dist;
    int npts_ramp = round(t_ramp / dt_);
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header;
    des_state.pose.pose = start_pose.pose; // start from here
    des_state.twist.twist = halt_twist_; // insist on starting from rest

    //ramp up omega (positive or negative);
    double t = 0.0;
    double accel = sgn(dpsi) * alpha_max_;
    double omega_des = 0.0;
    double psi_des = psi_start;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        omega_des = accel*t;
        des_state.twist.twist.angular.z = omega_des; //update rotation rate
        //update orientation
        psi_des = psi_start + 0.5 * accel * t*t;
        des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //now cruise for distance cruise_distance at const omega
    omega_des = sgn(dpsi)*omega_max_;
    des_state.twist.twist.angular.z  = sgn(dpsi)*omega_max_;
    double t_cruise = cruise_distance / omega_max_;
    int npts_cruise = round(t_cruise / dt_);
    for (int i = 0; i < npts_cruise; i++) {
        //Euler one-step integration
        psi_des += omega_des*dt_; //Euler one-step integration
        des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //ramp down omega to halt:
    for (int i = 0; i < npts_ramp; i++) {
        omega_des -= accel*dt_; //Euler one-step integration
        des_state.twist.twist.angular.z = omega_des;
        psi_des += omega_des*dt_; //Euler one-step integration
        des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where desired, and at rest:
    des_state.pose.pose = end_pose.pose; //
    des_state.twist.twist = halt_twist_; // insist on full stop
    vec_of_states.push_back(des_state);
}


// TRIANGULAR SPIN TRAJECTORY --------------------------------------------------
void TrajectoryBuilder::build_triangular_spin_traj( geometry_msgs::PoseStamped start_pose,
                                              geometry_msgs::PoseStamped end_pose,
                                              std::vector<nav_msgs::Odometry> &vec_of_states ){
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header;
    des_state.pose.pose = start_pose.pose; // start from here
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    vec_of_states.push_back(des_state);
    double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
    double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    double t_ramp = sqrt(fabs(dpsi) / alpha_max_);
    int npts_ramp = round(t_ramp / dt_);
    double psi_des = psi_start; // start from here
    double omega_des = 0.0; // assumes spin starts from rest;
    // position of des_state will not change; only orientation and twist
    double t = 0.0;
    double accel = sgn(dpsi) * alpha_max_; //watch out for sign: CW vs CCW rotation
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        omega_des = accel*t;
        des_state.twist.twist.angular.z = omega_des; // update rotation rate
        // update orientation
        psi_des = psi_start + 0.5 * accel * t*t;
        des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        omega_des -= accel*dt_; //Euler one-step integration
        des_state.twist.twist.angular.z = omega_des;
        psi_des += omega_des*dt_; //Euler one-step integration
        des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose; //start from here
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    vec_of_states.push_back(des_state);
}


// BUILD SPIN TRAJECTORY -------------------------------------------------------
void TrajectoryBuilder::build_spin_traj( geometry_msgs::PoseStamped start_pose,
                                   geometry_msgs::PoseStamped end_pose,
                                   std::vector<nav_msgs::Odometry> &vec_of_states ){
    //decide if triangular or trapezoidal profile:
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
    double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    ROS_INFO_STREAM("Rotational angle = " << dpsi*180/M_PI << " degrees");
    double ramp_up_time = omega_max_/ alpha_max_;
    double ramp_up_dist = 0.5 * alpha_max_ * ramp_up_time*ramp_up_time;
    //decide on triangular vs trapezoidal:
    if (fabs(dpsi) < 2.0 * ramp_up_dist) { //delta-angle is too short for trapezoid
        build_triangular_spin_traj(start_pose, end_pose, vec_of_states);
    } else {
        build_trapezoidal_spin_traj(start_pose, end_pose, vec_of_states);
    }
}


// TRAPEZOIDAL TRAVEL TRAJECTORY -----------------------------------------------
void TrajectoryBuilder::build_trapezoidal_travel_traj( geometry_msgs::PoseStamped start_pose,
                                                 geometry_msgs::PoseStamped end_pose,
                                                 std::vector<nav_msgs::Odometry> &vec_of_states ){
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_des = atan2(dy, dx);
    double trip_len = sqrt(dx * dx + dy * dy);
    double t_ramp = speed_max_ / accel_max_;
    double ramp_up_dist = 0.5 * accel_max_ * t_ramp*t_ramp;
    double cruise_distance = trip_len - 2.0 * ramp_up_dist; // distance to travel at v_max

    //start ramping up:
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header;
    des_state.pose.pose = start_pose.pose; //start from here
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    int npts_ramp = round(t_ramp / dt_);
    double x_des = x_start; // start from here
    double y_des = y_start;
    double speed_des = 0.0;
    des_state.twist.twist.angular.z = 0.0; //omega_des; will not change
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des); //constant
    // orientation of des_state will not change; only position and twist

    double t = 0.0;
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        speed_des = accel_max_*t;
        des_state.twist.twist.linear.x = speed_des; //update speed
        //update positions
        x_des = x_start + 0.5 * accel_max_ * t * t * cos(psi_des);
        y_des = y_start + 0.5 * accel_max_ * t * t * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    // now cruise for distance cruise_distance at const speed
    speed_des = speed_max_;
    des_state.twist.twist.linear.x = speed_des;
    double t_cruise = cruise_distance / speed_max_;
    int npts_cruise = round(t_cruise / dt_);

    for (int i = 0; i < npts_cruise; i++) {
        // Euler one-step integration
        x_des += speed_des * dt_ * cos(psi_des);
        y_des += speed_des * dt_ * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    // ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        speed_des -= accel_max_*dt_; // Euler one-step integration
        des_state.twist.twist.linear.x = speed_des;
        x_des += speed_des * dt_ * cos(psi_des); // Euler one-step integration
        y_des += speed_des * dt_ * sin(psi_des); // Euler one-step integration
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    // make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose;
    // but final orientation will follow from point-and-go direction
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    vec_of_states.push_back(des_state);
}


// TRIANGULAR TRAVEL TRAJECTORY ------------------------------------------------
void TrajectoryBuilder::build_triangular_travel_traj( geometry_msgs::PoseStamped start_pose,
                                                geometry_msgs::PoseStamped end_pose,
                                                std::vector<nav_msgs::Odometry> &vec_of_states ){
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_des = atan2(dy, dx);
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header;
    des_state.pose.pose = start_pose.pose;  // start from here
    des_state.twist.twist = halt_twist_;    // insist on starting from rest
    double trip_len = sqrt(dx * dx + dy * dy);
    double t_ramp = sqrt(trip_len / accel_max_);
    int npts_ramp = round(t_ramp / dt_);
    double v_peak = accel_max_*t_ramp;  // could consider special cases for reverse motion
    double d_vel = alpha_max_*dt_;      // incremental velocity changes for ramp-up

    double x_des = x_start; // start from here
    double y_des = y_start;
    double speed_des = 0.0;
    des_state.twist.twist.angular.z = 0.0; // omega_des; will not change
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des); // constant
    // orientation of des_state will not change; only position and twist
    double t = 0.0;
    // ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        speed_des = accel_max_*t;
        des_state.twist.twist.linear.x = speed_des; // update speed
        // update positions
        x_des = x_start + 0.5 * accel_max_ * t * t * cos(psi_des);
        y_des = y_start + 0.5 * accel_max_ * t * t * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    // ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        speed_des -= accel_max_*dt_; // Euler one-step integration
        des_state.twist.twist.linear.x = speed_des;
        x_des += speed_des * dt_ * cos(psi_des); // Euler one-step integration
        y_des += speed_des * dt_ * sin(psi_des); // Euler one-step integration
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    // make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose;
    // but final orientation will follow from point-and-go direction
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    vec_of_states.push_back(des_state);
}


// BUILD TRAVEL TRAJECTORY -----------------------------------------------------
void TrajectoryBuilder::build_travel_traj( geometry_msgs::PoseStamped start_pose,
                                     geometry_msgs::PoseStamped end_pose,
                                     std::vector<nav_msgs::Odometry> &vec_of_states ){
    // decide if triangular or trapezoidal profile:
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double trip_len = sqrt(dx * dx + dy * dy);
    double ramp_up_dist = 0.5 * speed_max_ * speed_max_ / alpha_max_;
    ROS_INFO_STREAM("Traslacional distance = " << trip_len << " meters");
    if (trip_len < 2.0 * ramp_up_dist) { // length is too short for trapezoid
        build_triangular_travel_traj(start_pose, end_pose, vec_of_states);
    } else {
        build_trapezoidal_travel_traj(start_pose, end_pose, vec_of_states);
    }
}


// BUILD POINT-AND-GO TRAJECTORY -----------------------------------------------
void TrajectoryBuilder::build_point_and_go_traj( geometry_msgs::PoseStamped start_pose,
                                           geometry_msgs::PoseStamped end_pose,
                                           std::vector<nav_msgs::Odometry> &vec_of_states ){
    ROS_INFO("Building point-and-go trajectory");
    nav_msgs::Odometry bridge_state;
    geometry_msgs::PoseStamped bridge_pose; // bridge end of prev traj to start of new traj
    vec_of_states.clear(); // get ready to build a new trajectory of desired states

    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double des_psi = atan2(dy, dx); // heading to point towards goal pose

    // bridge pose: state of robot with start_x, start_y, but pointing at next subgoal
    // achieve this pose with a spin move before proceeding to subgoal with translational
    // motion
    bridge_pose = start_pose;
    bridge_pose.pose.orientation = convertPlanarPsi2Quaternion(des_psi);
    //build trajectory to reorient
    build_spin_traj(start_pose, bridge_pose, vec_of_states);
    //start next segment where previous segment left off
    build_travel_traj(bridge_pose, end_pose, vec_of_states);
}
