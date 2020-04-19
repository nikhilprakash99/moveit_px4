// ROS core includes
#include <ros/ros.h>

// ROS action
#include <actionlib/server/action_server.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>

// ROS msg headers
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// threads
#include <pthread.h>

// typedefs
typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
typedef ActionServer::GoalHandle GoalHandle;
typedef trajectory_msgs::MultiDOFJointTrajectory Trajectory;

class Controller{

public:
	// class Constructor
	Controller(ros::NodeHandle &n):
		node_(n),
		action_server_(node_, "multirotor_trajectory", boost::bind(&Controller::goalCB, this, _1), boost::bind(&Controller::cancelCB, this, _1), false)
	{
		// status variables
		has_active_goal_ = false;
		created = false;

		ROS_INFO_STREAM("\nController Ready !!!\n");
	}

private:

	// class Handles and api
	ros::NodeHandle node_;
	ActionServer action_server_;
	GoalHandle active_goal_;

	// status variables
	bool has_active_goal_;
	bool created;

	// controllers and threads
	pthread_t trajectoryExecutor;

	// class data members
	Trajectory toExecute;

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
			//pub_topic.publish(empty);////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
			//pub_topic.publish(empty);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = false;
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
	void executeTrajectory(){}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "guidance_controller");
	ros::NodeHandle nh;

	Controller control(nh);

	ros::spin();

	return 0;
}