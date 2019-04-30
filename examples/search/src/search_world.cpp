#include <search.h>
#include <ros/ros.h>
#include <search/TrackObsAction.h>
#include <search_world.h>

// for tests
#include <iostream>

//using namespace despot;
namespace despot{

#define TAG 4
#define DEFAULT_NOISE_SIGMA 0.5
#define TERMINATION_OBSERVATION 101
#define NUM_LASER_DIRECTIONS 8

SearchWorld::SearchWorld(DSPOMDP* model, unsigned seed):model_(model)
{
    std::cout<<"search world constructor"<<std::endl;
    random_= Random(seed);

}
SearchWorld::~SearchWorld(){
    if (model_ != NULL)
    {
    
        delete model_;
        model_ = NULL;
    
    }


}

double SearchWorld::noise_sigma_ = DEFAULT_NOISE_SIGMA;

bool SearchWorld::Connect(){

	// initialize ROS node
	int argc;
	char ** argv;
	ros::init(argc, argv, "test_search");
	nh = ros::NodeHandlePtr(new ros::NodeHandle);
    std::cout<<"world_connect"<<std::endl;

	// get laser's noise sigma
	if (nh->getParam("/robot/hsrb_interface/noise", noise_sigma_))
	{
	  ROS_INFO("Initialized ith noise: %f stddev.", noise_sigma_);
	}
	else
	{
	  ROS_INFO("Initialized with default noise: %f stddev.", noise_sigma_);
	}

    //search_state=0;

	// wait for laser tag controller service to show up (blocking call)
	//ros::service::waitForService("laser_tag_action_obs", -1);

	// setup service client
	//client = nh->serviceClient<search::TagActionObs>("laser_tag_action_obs");

    std::cout<<"world_connect done"<<std::endl;
    return true;
}

//Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable
State* SearchWorld::Initialize(){
    std::cout<<"search world initialized"<<std::endl;
    state_ = model_->CreateStartState();
    return state_;
	//return NULL;
}

//Get the state of the system (only applicable for simulators or POMDP world)
State* SearchWorld::GetCurrentState() const {
    std::cout<<"get current state search_world"<<std::endl;
    //ROS_INFO("get current state");
    return state_;

}

//Send action to be executed by the system, receive observations terminal signals from the system
bool SearchWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs){

    ROS_INFO("search execute_action");

	//searchTigerState state; 
    actionlib::SimpleActionClient<search::TrackObsAction> ac_("search_obs", true);
    ac_.waitForServer();
    
    search::TrackObsGoal goal;
    goal.action = (int) action;
    ac_.sendGoal(goal);

    bool finished_timeout = ac_.waitForResult(ros::Duration(3.0));
    if(finished_timeout )
    {
        //actionlib::SimpleClientGoalState state =ac_.getSTate();
        ROS_INFO("Obs action succeed");
        //SimpleClientGoalState = ac_.getState();
        search::TrackObsResult result= *(ac_.getResult());
        //state.tiger_position =  result.state;
        obs = static_cast<OBS_TYPE>(result.observations);
        //step_reward_ = result.reward;
        //model_->reward = result.reward;)
        std::cout<<"obs from action"<<obs<<std::endl;
        //state_ = &state;
         //*ac_.getResult();
    }
    else
    {
        ROS_INFO("Obs_function did not finish before the time out");
    }

    //Should update state and reward
    bool terminal = dynamic_cast<Search*>(model_)->Step_world(*state_,random_.NextDouble(), action, step_reward_, obs);
    return terminal;

    //return 0;
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
}

