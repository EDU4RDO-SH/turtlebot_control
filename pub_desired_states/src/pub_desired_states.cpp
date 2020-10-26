#include <pub_desired_states.h>


// constructor -----------------------------------------------------------------
DesiredStates::DesiredStates(ros::NodeHandle &nh): nh_(nh){

    initializePublishers();     // initialize publishers
    initializeServices();       // initialize services
    loadParameters();           // load parameters from parameter server

    // configure trajectory builder
    trajectoryBuilder_.set_speed_max(speed_max_);
    trajectoryBuilder_.set_omega_max(omega_max_);
    trajectoryBuilder_.set_accel_max(accel_max_);
    trajectoryBuilder_.set_alpha_max(alpha_max_);
    trajectoryBuilder_.set_dt(dt);
    trajectoryBuilder_.set_path_move_tol_(path_move_tol);

    // define a halt state
    halt_twist_.linear.x = 0.0;
    halt_twist_.linear.y = 0.0;
    halt_twist_.linear.z = 0.0;
    halt_twist_.angular.x = 0.0;
    halt_twist_.angular.y = 0.0;
    halt_twist_.angular.z = 0.0;

    motion_mode_ = DONE_W_SUBGOAL;      // initial state of state machine

    current_pose_ = trajectoryBuilder_.xyPsi2PoseStamped(0, 0, 0);
    start_pose_ = current_pose_;
    end_pose_ = current_pose_;

    current_des_state_.twist.twist = halt_twist_;
    current_des_state_.pose.pose = current_pose_.pose;

    halt_state_ = current_des_state_;
    seg_start_state_ = current_des_state_;
    seg_end_state_ = current_des_state_;
}


// destructor ------------------------------------------------------------------
DesiredStates::~DesiredStates(){

}


// initialize publishers -------------------------------------------------------
void DesiredStates::initializePublishers() {
    ROS_INFO("Initializing Publishers ...");
    des_state_publisher_ = nh_.advertise<nav_msgs::Odometry>("/desired_states", 1, true);
    ROS_INFO("Done!");
}


// initialize services ---------------------------------------------------------
void DesiredStates::initializeServices() {
    ROS_INFO("Initializing Services ...");
    clean_path_queue_ = nh_.advertiseService("/clean_path_queue", &DesiredStates::cleanPathQueueCallback, this);
    append_path_queue_ = nh_.advertiseService("/append_path_queue", &DesiredStates::appendPathQueueCallback, this);
    ROS_INFO("Done!");
}


// function that loads parameters from parameter server ------------------------
void DesiredStates::loadParameters(){
    ROS_INFO("Getting parameters from Parameter Server ...");

    nh_.getParam("/speed_max", speed_max_);
    nh_.getParam("/omega_max", omega_max_);
    nh_.getParam("/accel_max", accel_max_);
    nh_.getParam("/alpha_max", alpha_max_);

    ROS_INFO("Done!");
}


// clean path queue service ----------------------------------------------------
bool DesiredStates::cleanPathQueueCallback( std_srvs::TriggerRequest &request,
                                            std_srvs::TriggerResponse &response){
    ROS_WARN("Cleaning path queue");

    while (!path_queue_.empty()){
        path_queue_.pop();
    }

    return true;
}


// append path queue service ---------------------------------------------------
bool DesiredStates::appendPathQueueCallback(pub_desired_states::pathRequest &request,
                                            pub_desired_states::pathResponse &response){

    int npts = request.path.poses.size();
    ROS_INFO("Appending new path to queue with %d points", npts);

    for (int i=0; i<npts; i++){
        path_queue_.push(request.path.poses[i]);
    }

    return true;
}


// set initial pose function ---------------------------------------------------
void DesiredStates::set_init_pose(double x, double y, double psi) {
    current_pose_ = trajectoryBuilder_.xyPsi2PoseStamped(x, y, psi);
}


// publish desired states state machine ----------------------------------------
void DesiredStates::pub_next_state(){

    // state machine; results in publishing a new desired state
    switch(motion_mode_){

        case PURSUING_SUBGOAL: //if have remaining pts in computed traj, send them
            current_des_state_ = des_state_vec_[traj_pt_i_];            // extract the i'th point of our plan
            current_pose_.pose = current_des_state_.pose.pose;
            current_des_state_.header.stamp = ros::Time::now();
            current_des_state_.header.frame_id = "des_states";
            des_state_publisher_.publish(current_des_state_);

            traj_pt_i_++; // increment counter to prep for next point of plan

            // check if we have clocked out all of our planned states:
            if(traj_pt_i_ >= npts_traj_){
                motion_mode_ = DONE_W_SUBGOAL;              // if so, indicate we are done
                seg_end_state_ = des_state_vec_.back();     // last state of traj

                if (!path_queue_.empty()) {
                    path_queue_.pop();                      // done w/ this subgoal; remove from the queue
                }

                ROS_INFO("Reached subgoal: x = %f, y = %f", current_pose_.pose.position.x, current_pose_.pose.position.y);
            }

            break;


        // suspended, pending a new subgoal, see if there is another subgoal in queue; if so,
        // use it to compute a new trajectory and change motion mode
        case DONE_W_SUBGOAL:

            if(!path_queue_.empty()){
                int n_path_pts = path_queue_.size();
                ROS_INFO("%d points in path queue", n_path_pts);
                start_pose_ = current_pose_;
                end_pose_ = path_queue_.front();
                trajectoryBuilder_.build_point_and_go_traj(start_pose_, end_pose_, des_state_vec_);
                traj_pt_i_ = 0;
                npts_traj_ = des_state_vec_.size();
                motion_mode_ = PURSUING_SUBGOAL;        // got a new plan; change mode to pursue it
                ROS_INFO("Computed new trajectory to pursue");
            }
            else { //no new goal? stay halted in this mode
                // by simply reiterating the last state sent (should have zero vel)
                des_state_publisher_.publish(seg_end_state_);
            }

        break;

        default:    // this should not happen
            ROS_WARN("Motion mode not recognized!");
            des_state_publisher_.publish(current_des_state_);
        break;

    }


}

// main program ----------------------------------------------------------------
int main(int argc, char **argv){

    ros::init(argc, argv, "desired_states_publisher_node");
    ros::NodeHandle nh;

    // instantiate a desired-state publisher object
    DesiredStates desiredStates(nh);

    // dt is set in header file pub_desired_states.h
    ros::Rate loop_rate(1/dt);                // timer for fixed publication rate
    desiredStates.set_init_pose(0, 0, 0);   // x=0, y=0, psi=0

    // main loop
    while(ros::ok()){
        desiredStates.pub_next_state();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
