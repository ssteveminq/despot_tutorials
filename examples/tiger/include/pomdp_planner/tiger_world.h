#include <tiger.h>
#include <despot/interface/world.h>

#include <tiger/TrackObsAction.h>
//#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
using namespace despot;

class TigerWorld: public World {
public:
    ros::NodeHandlePtr nh;
    //actionlib::SimpleActionClient<tiger::TrackObsAction> ac_;
    //ros::ServiceClient client;
    int tiger_state;

    virtual bool Connect();

    //Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable
    virtual State* Initialize();

    //Get the state of the system (only applicable for simulators or POMDP world)
    virtual State* GetCurrentState();

    //Send action to be executed by the system, receive observations terminal signals from the system
    virtual bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);

    static double noise_sigma_;
};
