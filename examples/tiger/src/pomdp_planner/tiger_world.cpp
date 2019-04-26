#include <tiger.h>
#include <ros/ros.h>
#include <tiger/TrackObsAction.h>
//#include <tiger/TrackObsGoal.h>
//#include <tiger/TrackObsResult.h>
#include <tiger_world.h>

// for tests
#include <iostream>

using namespace despot;

#define TAG 4
#define DEFAULT_NOISE_SIGMA 0.5
#define TERMINATION_OBSERVATION 101
#define NUM_LASER_DIRECTIONS 8

double TigerWorld::noise_sigma_ = DEFAULT_NOISE_SIGMA;

bool TigerWorld::Connect(){

	// initialize ROS node
	int argc;
	char ** argv;
	ros::init(argc, argv, "test_tiger");
	nh = ros::NodeHandlePtr(new ros::NodeHandle);
    std::cout<<"world_connect"<<std::endl;

	// get laser's noise sigma
	if (nh->getParam("/robot/hsrb_interface/noise", noise_sigma_))
	{
	  ROS_INFO("Initialized laser with noise: %f stddev.", noise_sigma_);
	}
	else
	{
	  ROS_INFO("Initialized laser with default noise: %f stddev.", noise_sigma_);
	}

    tiger_state=0;

	// wait for laser tag controller service to show up (blocking call)
	//ros::service::waitForService("laser_tag_action_obs", -1);

	// setup service client
	//client = nh->serviceClient<tiger::TagActionObs>("laser_tag_action_obs");

    std::cout<<"world_connect done"<<std::endl;
}

//Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable
State* TigerWorld::Initialize(){
	return NULL;
}

//Get the state of the system (only applicable for simulators or POMDP world)
State* TigerWorld::GetCurrentState(){
    return NULL;

	//return NULL;
}

//Send action to be executed by the system, receive observations terminal signals from the system
bool TigerWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs){

    ROS_INFO("execute_action");

    actionlib::SimpleActionClient<tiger::TrackObsAction> ac_("tiger_obs", true);
    ac_.waitForServer();
    
    tiger::TrackObsGoal goal;
    goal.action = (int) action;
    ac_.sendGoal(goal);

    bool finished_timeout = ac_.waitForResult(ros::Duration(10.0));
    if(finished_timeout )
    {
        //actionlib::SimpleClientGoalState state =ac_.getSTate();
        ROS_INFO("Obs action succeed");
        //SimpleClientGoalState = ac_.getState();
        tiger::TrackObsResult result= *(ac_.getResult());
        tiger_state = result.state;
         //*ac_.getResult();
    
    }
    else
    {
        ROS_INFO("Obs_function did not finish before the time out");
    
    }

    return 0;
	/* laser_tag::TagActionObs is a ROS service that takes in an action (integer) 
	 * and outputs observations (8 intergers) after executing the action. If the 
	 * 'Tag' action is called, it returns a boolean 'tag_success' with the outcome.
	 */
    /*

	laser_tag::TagActionObs srv;
	srv.request.action = (int) action; // actions: 0 - North, 1 - East, 2 - South, 3 - West, 4 - Tag
	if (client.call(srv))
	{

	  	// successful tag
		if(action == TAG && srv.response.tag_success == true)
		{
			obs=(OBS_TYPE)0;
			for (int dir = 0; dir < NUM_LASER_DIRECTIONS; dir++) {
				LaserTag::SetReading(obs, TERMINATION_OBSERVATION, dir);
			}
			return 1; // exit
		}
		// continue to observe
		else 
		{
			// observations after executing action
			std::vector<int> laser_obs = srv.response.observations;

			// print observations 
			ROS_INFO("Laser Observations");
			ROS_INFO("North: %d"    , laser_obs[0]);
			ROS_INFO("East: %d"     , laser_obs[1]);
			ROS_INFO("South: %d"    , laser_obs[2]);
			ROS_INFO("West: %d"     , laser_obs[3]);
			ROS_INFO("NorthEast: %d", laser_obs[4]);
			ROS_INFO("SouthEast: %d", laser_obs[5]);
			ROS_INFO("SouthWest: %d", laser_obs[6]);
			ROS_INFO("NorthWest: %d", laser_obs[7]);

			for (int dir = 0; dir < 8; dir++) {
			LaserTag::SetReading(obs, laser_obs[dir], dir);
			}
			return 0; // continue
		}
	}
	else
	{
		ROS_ERROR("Failed to execute action & receive observations. Something went wrong with the robot controller!");
		return 0; // continue
	}
    */
}

