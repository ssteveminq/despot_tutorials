#include <search.h>
#include <despot/core/globals.h>
#include <despot/interface/world.h>
#include <despot/interface/pomdp.h>
#include <despot/util/random.h>

#include <search/TrackObsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
//using namespace despot;

namespace despot{

class SearchWorld: public World {
protected:
        DSPOMDP* model_;
        Random random_;

public:
    ros::NodeHandlePtr nh;

    double step_reward_;
    //int search_state;
    SearchWorld(DSPOMDP* model, unsigned seed);
    virtual ~SearchWorld();

    virtual bool Connect();

    //Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable
    virtual State* Initialize();

    //Get the state of the system (only applicable for simulators or POMDP world)
    virtual State* GetCurrentState() const;

    //Send action to be executed by the system, receive observations terminal signals from the system
    virtual bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);

    static double noise_sigma_;
};
}
