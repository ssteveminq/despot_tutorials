#include "search.h"

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {

const ACT_TYPE Search::SEARCH = 0; //action for searching
const ACT_TYPE Search::MOVE = 1; //action for avoiding occlusion
const ACT_TYPE Search::WAIT = 2; //action for wait
const double Search::NOISE = 0.15;

/* =============================================================================
 * SearchState class
 * =============================================================================*/

SearchState::SearchState() :
	object_position(0) {
}

SearchState::SearchState(int position) :
	object_position(position) {
}

string SearchState::text() const {
    if(object_position == Search::VISIBLE)
        return "Visible";
    else if(object_position == Search::OCCLUSION)
        return "OCCLUSION";
    else if (object_position == Search::MISSING)
        return "MISSING";
    else
        return "state is Wrong: Check the state";
}

/* =============================================================================
 * OptimalSearchPolicy class
 * =============================================================================*/

class OptimalSearchPolicy: public DefaultPolicy {
public:
	OptimalSearchPolicy(const DSPOMDP* model,
		ParticleLowerBound* bound) :
		DefaultPolicy(model, bound) {
	}

	// NOTE: optimal for noise = 0.15
	ACT_TYPE Action(const vector<State*>& particles, RandomStreams& streams,
		History& history) const {
		/*
		 if (history.Size() == 0 || history.LastAction() != WAIT) {
		 actions->push_back(WAIT);
		 return actions;
		 }

		 actions->push_back(history.LastObservation());
		 */
		int count_diff = 0;
		for (int i = history.Size() - 1;
			i >= 0 && history.Action(i) == Search::WAIT; i--)
			count_diff += history.Observation(i) == Search::SEARCH ? 1 : -1;

		if (count_diff >= 2)
			return Search::MOVE;
		else if (count_diff <= -2)
			return Search::SEARCH;
		else
			return Search::WAIT;
	}
};

/* =============================================================================
 * Search class
 * =============================================================================*/
Search::Search() {
}

//should update state via transition proababilities
bool Search::Step_world( State& s,double random_num,  ACT_TYPE action, double& reward,
    OBS_TYPE obs) const {

	SearchState& state = static_cast<SearchState&>(s);
    std::cout<<"Step_world"<<std::endl;

	bool terminal = false;
    if(state.object_position == Search::VISIBLE)
    {
        //reward=20;
        if (action == SEARCH) {

            //if(random_num<0.05)
                //state.object_position = OCCLUSION ;
            if(random_num<0.25)
                state.object_position = MISSING;
            else
                state.object_position = VISIBLE;
        }
        else if(action == MOVE){
        
            if(random_num<0.5)
                state.object_position = MISSING;
            else
                state.object_position = VISIBLE;
        }
        else if(action == WAIT)
        {
            if(random_num<0.05)
                state.object_position = MISSING;
            else if(random_num<0.1)
                state.object_position = OCCLUSION ;
            else
                state.object_position = VISIBLE;

        }

    }
    else if(state.object_position == Search::OCCLUSION){

        if (action == SEARCH) {
            if(random_num<0.15)
                state.object_position = VISIBLE;
            else
                state.object_position = OCCLUSION;
        }
        else if(action == MOVE)
        {
            if(random_num<0.2)
                state.object_position = OCCLUSION;
            else
                state.object_position = VISIBLE;
        }
        else if(action == WAIT)
        {
            if(random_num<0.2)
                state.object_position = OCCLUSION;
            else
                state.object_position = VISIBLE;
        }
        reward=0;
    }
    else{//state == MISSING

        if (action == SEARCH) {
            if(random_num<0.2)
                state.object_position = MISSING;
            else
                state.object_position = VISIBLE;
        }
        else if(action == MOVE)
        {
            if(random_num<0.15)
                state.object_position = VISIBLE;
            else
                state.object_position = MISSING;
        }
        else if(action == WAIT)
        {
            if(random_num<0.2)
                state.object_position = VISIBLE;
            else
                state.object_position = MISSING;
        }
        
        reward=0;
    }


    //if(obs == Search::TARGET)
    //{
        //reward=20;
        //state.object_position = random_num <= 0.95 ? VISIBLE: MISSING;
    //}
    //else{
        //state.object_position = random_num <= 0.6 ? OCC_NOTARGET: MISSING;
        //if (action == SEARCH || action == MOVE) {
                //reward=5;
        //}
        //else{
            //reward=-1;
        //}
    //}

    return terminal;
} 

//should update state and observe
bool Search::Step(State& s, double random_num, ACT_TYPE action, double& reward,
	OBS_TYPE& obs) const {
    //std::cout<<"random_num"<<random_num<<std::endl;
    
	SearchState& state = static_cast<SearchState&>(s);
	bool terminal = false;

    if(state.object_position == Search::VISIBLE)
    {
        //reward=20;
        if (action == SEARCH) {

            //if(random_num<0.05)
                //state.object_position = OCCLUSION ;
            if(random_num<0.25)
                state.object_position = MISSING;
            else
                state.object_position = VISIBLE;

            reward=-30;
        }
        else if(action == MOVE){
        
            if(random_num<0.5)
                state.object_position = MISSING;
            else
                state.object_position = VISIBLE;

            reward=-30;
        }
        else if(action == WAIT)
        {
            if(random_num<0.05)
                state.object_position = MISSING;
            else if(random_num<0.1)
                state.object_position = OCCLUSION ;
            else
                state.object_position = VISIBLE;

            reward=10;

        }

    }
    else if(state.object_position == Search::OCCLUSION){

        if (action == SEARCH) {
            if(random_num<0.15)
                state.object_position = VISIBLE;
            else
                state.object_position = OCCLUSION;

            reward=-20;
        }
        else if(action == MOVE)
        {
            if(random_num<0.2)
                state.object_position = OCCLUSION;
            else
                state.object_position = VISIBLE;

            reward=50;
        }
        else if(action == WAIT)
        {
            if(random_num<0.2)
                state.object_position = OCCLUSION;
            else
                state.object_position = VISIBLE;

            reward=-5;
        }
        //reward=0;
    }
    else{//state == MISSING

        if (action == SEARCH) {
            if(random_num<0.2)
                state.object_position = MISSING;
            else
                state.object_position = VISIBLE;

            reward=50;
        }
        else if(action == MOVE)
        {
            if(random_num<0.15)
                state.object_position = VISIBLE;
            else
                state.object_position = MISSING;

            reward=-20;
        }
        else if(action == WAIT)
        {
            if(random_num<0.2)
                state.object_position = VISIBLE;
            else
                state.object_position = MISSING;

            reward=-5;
        }
        
        //reward=0;
    }

    if (random_num <= NOISE)
        obs = 2-state.object_position;
    else
        obs = state.object_position;

	return terminal;
}

int Search::NumStates() const {
	return 3;
}

int Search::NumActions() const {
	return 3;
}


double Search::Reward(const State& s,  ACT_TYPE action) const
{
    std::cout<<"search reward function"<<std::endl;
    double random_num = Random::RANDOM.NextDouble();

    const SearchState* state= static_cast<const SearchState*>(&s);
    double reward = 0;

    if(state->object_position == Search::VISIBLE)
    {
        //reward=20;
        if (action == SEARCH) {


            reward=-30;
            //if(random_num<0.25){
                //state->object_position = MISSING;
                //reward=-30;
            //}
            //else{
                //state->object_position = VISIBLE;
                //reward=+1;
            //}
        }
        else if(action == MOVE){
        
            reward=-30;
            //if(random_num<0.5){
                //state->object_position = MISSING;
                //reward=-30;
            //}
            //else{
                //state->object_position = VISIBLE;
                //reward=1;
            //}

        }
        else if(action == WAIT)
        {

            reward=10;
            //if(random_num<0.05){
                //state->object_position = MISSING;
                //reward=0;
            //}
            //else if(random_num<0.1){
                //state->object_position = OCCLUSION ;
                //reward=0;
            //}
            //else{
                //state->object_position = VISIBLE;
                //reward=10;
            //}

        }

    }
    else if(state->object_position == Search::OCCLUSION){

        if (action == SEARCH) {
            //if(random_num<0.15){
                //state->object_position = VISIBLE;
            //}
            //else
            //{
                //state->object_position = OCCLUSION;
            //}
            reward=-50;
        }
        else if(action == MOVE)
        {
            //if(random_num<0.2)
            //{
                //state->object_position = OCCLUSION;
                //reward=1;
            //}
            //else{
                //state->object_position = VISIBLE;
                //reward=50;
            //}
            //
                reward=50;
        }
        else if(action == WAIT)
        {
            //if(random_num<0.2){
                //state->object_position = OCCLUSION;

            //}
            //else{
                //state->object_position = VISIBLE;
            //}

            reward=-1;
        }
    }
    else{//state == MISSING

        if (action == SEARCH) {
            //if(random_num<0.2){
                //state->object_position = MISSING;
                //reward=1;
            //}
            //else{
                //state->object_position = VISIBLE;
                //reward=50;
            //}
            //
                reward=50;
        }
        else if(action == MOVE)
        {
            //if(random_num<0.15){
                //state->object_position = VISIBLE;
            //}
            //else{
                //state->object_position = MISSING;
            //}
            reward=-50;
        }
        else if(action == WAIT)
        {
            //if(random_num<0.2){
                //state->object_position = VISIBLE;
            //}
            //else{
                //state->object_position = MISSING;
            //}

            reward=-1;
        }
    }

    return reward;
}


double Search::Reward(int s, ACT_TYPE a) const
{
    //std::cout<<"reward function"<<std::endl;
    //ROS_INFO("reward");
    double reward = 0;
    if (a== SEARCH || a== MOVE) {
        reward =s!= a? 10 : -100;
    } else {
        reward = -1;
	}
    return reward;
}



double Search::ObsProb(OBS_TYPE obs, const State& s, ACT_TYPE action) const {
	const SearchState& state = static_cast<const SearchState&>(s);
    double prob=0.0;

    if(state.object_position == Search::VISIBLE)
    {
        if (action == SEARCH) {
            if(obs == TARGET)
                prob =0.8;
            else if (obs == MISS_NOTARGET)
                prob = 0.2;
        }
        else if(action == MOVE){
            if(obs == TARGET)
                prob =0.2;
            else if (obs == MISS_NOTARGET)
                prob = 0.4;
            else
                prob = 0.4;
        }
        else if(action == WAIT)
        {
            if(obs == TARGET)
                prob =0.9;
            else if (obs == MISS_NOTARGET)
                prob = 0.05;
            else
                prob = 0.05;
        }

    }
    else if(state.object_position == Search::OCCLUSION){

        if (action == SEARCH) {
            if(obs == TARGET)
                prob =0.1;
            else if (obs == OCC_NOTARGET)
                prob = 0.9;
        }
        else if(action == MOVE)
        {
            if(obs == TARGET)
                prob =0.9;
            else if (obs == OCC_NOTARGET)
                prob = 0.1;
        }
        else if(action == WAIT)
        {
            if(obs == TARGET)
                prob =0.15;
            else if (obs == OCC_NOTARGET)
                prob = 0.85;
        }
    }
    else{//state == MISSING

        if (action == SEARCH) {

            if(obs == TARGET)
                prob =0.9;
            else if (obs == MISS_NOTARGET)
                prob = 0.1;
        }
        else if(action == MOVE)
        {
            if(obs == TARGET)
                prob =0.1;
            else if (obs == MISS_NOTARGET)
                prob = 0.9;
        }
        else if(action == WAIT)
        {
            if(obs == TARGET)
                prob =0.15;
            else if (obs == MISS_NOTARGET)
                prob = 0.85;
        }
    }
    return prob;

}

State* Search::CreateStartState(string type) const {
	return new SearchState(Random::RANDOM.NextInt(3));
}

Belief* Search::InitialBelief(const State* start, string type) const {
    std::cout<<"search_belief"<<std::endl;
	vector<State*> particles;
	SearchState* left = static_cast<SearchState*>(Allocate(-1, 0.5));
	left->object_position = SEARCH;
	particles.push_back(left);
	SearchState* right = static_cast<SearchState*>(Allocate(-1, 0.5));
	right->object_position = MOVE;
	particles.push_back(right);
	return new ParticleBelief(particles, this);
}

ScenarioLowerBound* Search::CreateScenarioLowerBound(string name,
	string particle_bound_name) const {
	ScenarioLowerBound* bound = NULL;
	if (name == "TRIVIAL" || name == "DEFAULT") {
		bound = new TrivialParticleLowerBound(this);
	} else if (name == "RANDOM") {
		bound = new RandomPolicy(this,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "SEARCH") {
		bound = new BlindPolicy(this, SEARCH,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "MOVE") {
		bound = new BlindPolicy(this, MOVE,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "WAIT") {
		bound = new BlindPolicy(this, WAIT,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "OPTIMAL") {
		bound = new OptimalSearchPolicy(this,
			CreateParticleLowerBound(particle_bound_name));
	} else {
		cerr << "Unsupported scenario lower bound: " << name << endl;
		exit(1);
	}
	return bound;
}

void Search::PrintState(const State& state, ostream& out) const {
	const SearchState& searchstate = static_cast<const SearchState&>(state);
	out << searchstate.text() << endl;
}

void Search::PrintBelief(const Belief& belief, ostream& out) const {
}

void Search::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
	if (obs == TARGET ) {
		out << "TARGET " << endl;
	} else if (obs == OCC_NOTARGET) {
		out << "OCC_NOTARGET" << endl;
	} else {
		out << "MISS_NOTARGET" << endl;
	}


	out << (obs == SEARCH ? "SEARCH" : "MOVE") << endl;
}

void Search::PrintAction(ACT_TYPE action, ostream& out) const {
	if (action == SEARCH) {
		out << "SEARCH" << endl;
	} else if (action == MOVE) {
		out << "MOVE_SEE" << endl;
	} else {
		out << "WAIT" << endl;
	}
}

State* Search::Allocate(int state_id, double weight) const {
	SearchState* particle = memory_pool_.Allocate();
	particle->state_id = state_id;
	particle->weight = weight;
	return particle;
}

State* Search::Copy(const State* particle) const {
	SearchState* new_particle = memory_pool_.Allocate();
	*new_particle = *static_cast<const SearchState*>(particle);
	new_particle->SetAllocated();
	return new_particle;
}

void Search::Free(State* particle) const {
	memory_pool_.Free(static_cast<SearchState*>(particle));
}

int Search::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

} // namespace despot
