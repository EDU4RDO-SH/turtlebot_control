#include <turtlebot_linear_control.h>

// constructor -----------------------------------------------------------------
LinearController::LinearController(ros::NodeHandle &nh): nh_(nh){
    initializeSubscribers();
    initializePublishers();

    // Initialize member variables
    des_states_ = current_odom_;

    // Make sure the speed values are set to zero
    current_speed_des_ = 0.0;
    current_omega_des_ = 0.0;
    des_states_.twist.twist.linear.x = current_speed_des_;
    des_states_.twist.twist.angular.z = current_omega_des_;

    des_states_.header.stamp = ros::Time::now();

    // Initialize the twist command components to zero
    vel_cmd_.linear.x = 0.0;
    vel_cmd_.linear.y = 0.0;
    vel_cmd_.linear.z = 0.0;
    vel_cmd_.angular.x = 0.0;
    vel_cmd_.angular.y = 0.0;
    vel_cmd_.angular.z = 0.0;

    // initialize twist stamped message
    vel_cmd2_.twist = vel_cmd_;
    vel_cmd2_.header.stamp = ros::Time::now();

    // state errors stamp
    state_errors_.header.stamp = ros::Time::now();
}


// destructor ------------------------------------------------------------------
LinearController::~LinearController(){

}


// initialize subscribers ------------------------------------------------------
void LinearController::initializeSubscribers(){
    ROS_INFO("Initializing Subscribers ...");
    odom_sub_ = nh_.subscribe("/odom", 1, &LinearController::odomCallback, this);
    des_states_sub_ = nh_.subscribe("/desired_states", 1, &LinearController::desStatesCallback, this);
    ROS_INFO("Done!");

}


// initialize publishers -------------------------------------------------------
void LinearController::initializePublishers(){
    ROS_INFO("Initializing Publishers ...");
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    vel_pub2_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1, true);
    errs_pub_ =  nh_.advertise<std_msgs::Float32MultiArray>("/steering_errors", 1, true);
    state_errors_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/state_errors", 1, true);
    ROS_INFO("Done!");
}


// current states callback function --------------------------------------------
void LinearController::odomCallback(const nav_msgs::Odometry &odom_msg){
    current_odom_ = odom_msg;
    odom_vel_ = odom_msg.twist.twist.linear.x;
    odom_omega_ = odom_msg.twist.twist.angular.z;
    odom_x_ = odom_msg.pose.pose.position.x;
    odom_y_ = odom_msg.pose.pose.position.y;
    odom_quat_ = odom_msg.pose.pose.orientation;
    odom_psi_ = convertPlanarQuat2Psi(odom_quat_);
}


// desired states callback function --------------------------------------------
void LinearController::desStatesCallback(const nav_msgs::Odometry &des_states_msg){
    des_states_ = des_states_msg;
    des_states_vel_ = des_states_msg.twist.twist.linear.x;
    des_states_omega_ = des_states_msg.twist.twist.angular.z;
    des_states_x_ = des_states_msg.pose.pose.position.x;
    des_states_y_ = des_states_msg.pose.pose.position.y;
    des_states_quat_ = des_states_msg.pose.pose.orientation;
    des_states_psi_ = convertPlanarQuat2Psi(des_states_quat_);
}


// compute minimum angle -------------------------------------------------------
double LinearController::min_dang(double dang){

    while(dang > M_PI){
        dang = dang - 2.0*M_PI;
    }

    while(dang < -M_PI){
        dang = dang + 2.0*M_PI;
    }

    return dang;
}


// saturation function ---------------------------------------------------------
double LinearController::sat(double x){

    if(x > 1.0){
        return 1.0;
    }

    if(x < -1.0){
        return -1.0;
    }

    return x;
}


// convert from quaternion to heading ------------------------------------------
double LinearController::convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion){
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double psi = 2.0*atan2(quat_z, quat_w);
    return psi;
}


// control algorithm -----------------------------------------------------------
void LinearController::control_algorithm(){

    double controller_speed;
    double controller_omega;

    double tx = cos(des_states_psi_);
    double ty = sin(des_states_psi_);
    double nx = -ty;
    double ny = tx;


    // Errors
    double heading_err;
    double lateral_err;
    double trip_dist_err;

    double dx = des_states_x_ - odom_x_;
    double dy = des_states_y_ - odom_y_;

    lateral_err = dx*nx + dy*ny;
    trip_dist_err = dx*tx + dy*ty;
    heading_err = min_dang(des_states_psi_ - odom_psi_);

    // publish errors for debugging
    errs_.data.clear();
    errs_.data.push_back(lateral_err);
    errs_.data.push_back(trip_dist_err);
    errs_.data.push_back(heading_err);
    errs_pub_.publish(errs_);

    // publish state errors
    state_errors_.header.stamp = ros::Time::now();
    state_errors_.point.x = dx;
    state_errors_.point.y = dy;
    state_errors_.point.z = heading_err;
    state_errors_pub_.publish(state_errors_);


    // Applying the control law
    controller_speed = des_states_vel_ + K_TRIP_DIST * trip_dist_err;

    controller_omega = des_states_omega_ + K_PSI * heading_err + K_DISP * lateral_err;
    controller_omega = MAX_OMEGA * sat(controller_omega / MAX_OMEGA);

    // Sending out the computed control signals
    vel_cmd_.linear.x = controller_speed;
    vel_cmd_.angular.z = controller_omega;

    vel_cmd2_.twist = vel_cmd_;
    vel_cmd2_.header.stamp = ros::Time::now();

    vel_pub_.publish(vel_cmd_);
    vel_pub2_.publish(vel_cmd2_);

}


// main program ----------------------------------------------------------------
int main(int argc, char **argv) {

    ros::init(argc, argv, "turtlebot_linear_controller_node");
    ros::NodeHandle nh;

    ROS_INFO("Instantiating an object of type LinearController");
    LinearController linearController(nh);
    ros::Rate loop_rate(UPDATE_RATE);

    while (ros::ok()) {
        linearController.control_algorithm();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
