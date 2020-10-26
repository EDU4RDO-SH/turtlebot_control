#ifndef PUB_DESIRED_STATES_H_
#define PUB_DESIRED_STATES_H_

// has almost all the libraries we need
#include <trajectory_builder.h>

// ROS messages
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

// ROS services
#include <std_srvs/Trigger.h>
#include <pub_desired_states/path.h>

// global variables
const double dt = 0.02;
const double path_move_tol = 0.01;

// state machine keywords
const int DONE_W_SUBGOAL = 1;
const int PURSUING_SUBGOAL = 2;

// class definition
class DesiredStates{

private:
    ros::NodeHandle nh_;

    // publisher
    ros::Publisher des_state_publisher_;

    // services
    ros::ServiceServer clean_path_queue_;
    ros::ServiceServer append_path_queue_;

    // member variables
    std::vector<nav_msgs::Odometry> des_state_vec_;
    std::queue<geometry_msgs::PoseStamped> path_queue_;


    nav_msgs::Odometry halt_state_;
    nav_msgs::Odometry seg_end_state_;
    nav_msgs::Odometry seg_start_state_;
    nav_msgs::Odometry current_des_state_;

    geometry_msgs::Twist halt_twist_;
    geometry_msgs::PoseStamped start_pose_;
    geometry_msgs::PoseStamped end_pose_;
    geometry_msgs::PoseStamped current_pose_;

    int motion_mode_;
    int traj_pt_i_;
    int npts_traj_;

    // dynamic parameters:
    double speed_max_;
    double omega_max_;
    double accel_max_;
    double alpha_max_;

    // trajectory builder object;
    TrajectoryBuilder trajectoryBuilder_;

    // member methods:
    void initializePublishers();
    void initializeServices();
    void loadParameters();

    bool cleanPathQueueCallback( std_srvs::TriggerRequest &request,
                                 std_srvs::TriggerResponse &response );

    bool appendPathQueueCallback( pub_desired_states::pathRequest &request,
                                  pub_desired_states::pathResponse &response );

public:

    DesiredStates(ros::NodeHandle &nh);     // constructor
    ~DesiredStates();                       // destructor

    void set_init_pose(double x, double y, double psi);
    void pub_next_state();

};  // end of class definition

#endif
