// ROS core includes
#include <ros/ros.h>

// ROS action
#include <actionlib/server/action_server.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>

// ROS msg libraries
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

// C++ Includes
#include <pthread.h>
#include <cmath>
#include <vector>

#define GOAL_THRESHOLD 0.2
#define PI 3.141592

// typedefs
typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
typedef ActionServer::GoalHandle GoalHandle;
typedef trajectory_msgs::MultiDOFJointTrajectory Trajectory;

class Controller{

public:
	// class Constructor
	Controller(ros::NodeHandle &n):
		node_(n),
		action_server_(node_, "multirotor_trajectory", boost::bind(&Controller::goalCB, this, _1), boost::bind(&Controller::cancelCB, this, _1), false),
		rate(20)
	{
		// status variables
		has_active_goal_ = false;
		created = false;

		// start publishers and subscribers
		state_sub = node_.subscribe<mavros_msgs::State>("mavros/state", 10, &Controller::state_cb, this);
		pose_sub = node_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Controller::pose_cb, this);
		local_pos_pub = node_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

		// start services
		arming_client = node_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    	set_mode_client = node_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

		ROS_INFO_STREAM("\nController Ready !!!\n");
		ROS_INFO_STREAM("testing guidance controller");

		runtest();
	}

private:

	// class handles and api
	ros::NodeHandle node_;
	ActionServer action_server_;
	GoalHandle active_goal_;
	ros::Subscriber state_sub;
	ros::Subscriber pose_sub;
	ros::Publisher local_pos_pub;
	ros::ServiceClient arming_client;
	ros::ServiceClient set_mode_client;
	ros::Rate rate;

	// controllers and threads
	pthread_t trajectoryExecutor;

	// class data members
	Trajectory toExecute;
	geometry_msgs::PoseStamped des_wp;
	geometry_msgs::PoseStamped cur_pose;

	// status variables
	bool has_active_goal_;
	bool created;
	mavros_msgs::State current_state;

	// member functions
	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(created){
				ROS_INFO_STREAM("thread stopped due cancel request\n");
				pthread_cancel(trajectoryExecutor);
				created=false;
			}

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
	}

	void goalCB(GoalHandle gh){
		if (has_active_goal_)
		{
			// Stops the controller.
			if(created)
			{
				ROS_INFO_STREAM("old thread stopped due to new goal\n");
				pthread_cancel(trajectoryExecutor);
				created=false;
			}

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = true;
		toExecute = gh.getGoal()->trajectory;

		if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0)
		{
			created=true;
			ROS_INFO_STREAM("Thread for trajectory execution created \n");
		} else {
			ROS_ERROR_STREAM("Thread creation failed! \n");
		}
	}

	// running the 2nd thread
	static void* threadWrapper(void* arg) {
		Controller * mySelf=(Controller*)arg;
		mySelf->executeTrajectory();
		return NULL;
	}

	// Guidance Algorithm
	void executeTrajectory(){

		// waiting for FCU connection
		while(ros::ok() && !current_state.connected){
        	ros::spinOnce();
        	rate.sleep();
    	}

    	copywp(toExecute.points[0],des_wp);

    	//send a few setpoints before starting
	    for(int i = 100; ros::ok() && i > 0; --i){
	        local_pos_pub.publish(des_wp);
	        ros::spinOnce();
	        rate.sleep();
	    }

	    mavros_msgs::SetMode offb_set_mode;
	    offb_set_mode.request.custom_mode = "OFFBOARD";

	    mavros_msgs::CommandBool arm_cmd;
	    arm_cmd.request.value = true;

	    ros::Time last_request = ros::Time::now();

	    int wp_i = 0; 
	    int wp_size = toExecute.points.size();

	    while(ros::ok()){
	        if( current_state.mode != "OFFBOARD" &&
	            (ros::Time::now() - last_request > ros::Duration(5.0))){
	            if( set_mode_client.call(offb_set_mode) &&
	                offb_set_mode.response.mode_sent){
	                ROS_INFO("Offboard enabled");
	            }
	            last_request = ros::Time::now();
	        } else {
	            if( !current_state.armed &&
	                (ros::Time::now() - last_request > ros::Duration(5.0))){
	                if( arming_client.call(arm_cmd) &&
	                    arm_cmd.response.success){
	                    ROS_INFO("Vehicle armed");
	                }
	                last_request = ros::Time::now();
	            }
	        }

	        local_pos_pub.publish(des_wp);

	        ROS_INFO("Current WP: X:%0.2f  Y: %0.2f Z: %0.2f",des_wp.pose.position.x,des_wp.pose.position.y,des_wp.pose.position.z);

	        if(dist_to_goal() < GOAL_THRESHOLD)
	        {
	        	if(ros::Time::now() > toExecute.header.stamp + toExecute.points[wp_i].time_from_start)
	        	{
	        		if( ++wp_i < wp_size)
	        			copywp(toExecute.points[wp_i],des_wp);
	        		else
	        			break;
	        	}
	        }

	        ros::spinOnce();
	        rate.sleep();
	    }

	    active_goal_.setSucceeded();
		has_active_goal_=false;
		created=false;	    
	}

	void copywp(trajectory_msgs::MultiDOFJointTrajectoryPoint wp, geometry_msgs::PoseStamped& des_wp)
	{
		des_wp.pose.position.x = wp.transforms[0].translation.x;
		des_wp.pose.position.y = wp.transforms[0].translation.y;
		des_wp.pose.position.z = wp.transforms[0].translation.z;
		des_wp.pose.orientation.x = wp.transforms[0].rotation.x;
		des_wp.pose.orientation.y = wp.transforms[0].rotation.y;
		des_wp.pose.orientation.z = wp.transforms[0].rotation.z;
		des_wp.pose.orientation.w = wp.transforms[0].rotation.w;
	}

	double dist_to_goal(){
		double del_x = des_wp.pose.position.x - cur_pose.pose.position.x;
		double del_y = des_wp.pose.position.y - cur_pose.pose.position.y;
		double del_z = des_wp.pose.position.z - cur_pose.pose.position.z;
		return sqrt(del_x*del_x + del_y*del_y + del_z*del_z);
	}

	void state_cb(const mavros_msgs::State::ConstPtr& msg){
    	current_state = *msg;
	}

	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    	cur_pose = *msg;
	}

	void runtest(){
		trajectory_msgs::MultiDOFJointTrajectoryPoint point;
		geometry_msgs::Transform transform;
		double raidius = 1;
		double height = 1;

		for ( int i=0;i<200;i++)
		{
			transform.translation.x = raidius*sin(6*PI*i/200);
			transform.translation.y = raidius*sin(6*PI*i/200);
			transform.translation.z = i*0.1;
			point.transforms.push_back(transform);
			point.time_from_start = ros::Duration(i*0.2);

			toExecute.points.push_back(point);

		}

		toExecute.header.stamp = ros::Time::now()+ros::Duration(1);
		ROS_INFO("executing Trajectory");
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "guidance_controller");
	ros::NodeHandle nh;

	Controller control(nh);

	ros::spin();

	return 0;
}