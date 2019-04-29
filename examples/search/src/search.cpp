#include "search.h"

#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {

const ACT_TYPE Search::LEFT = 0;
const ACT_TYPE Search::RIGHT = 1;
const ACT_TYPE Search::LISTEN = 2;
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
	return object_position == Search::LEFT ? "LEFT" : "RIGHT";
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
		 if (history.Size() == 0 || history.LastAction() != LISTEN) {
		 actions->push_back(LISTEN);
		 return actions;
		 }

		 actions->push_back(history.LastObservation());
		 */

		int count_diff = 0;
		for (int i = history.Size() - 1;
			i >= 0 && history.Action(i) == Search::LISTEN; i--)
			count_diff += history.Observation(i) == Search::LEFT ? 1 : -1;

		if (count_diff >= 2)
			return Search::RIGHT;
		else if (count_diff <= -2)
			return Search::LEFT;
		else
			return Search::LISTEN;
	}
};

/* =============================================================================
 * Search class
 * =============================================================================*/

Search::Search() {
}

bool Search::Step_world( State& s,double random_num,  ACT_TYPE action, double& reward,
    OBS_TYPE obs) const {

	SearchState& state = static_cast<SearchState&>(s);
    std::cout<<"Step_world"<<std::endl;

	bool terminal = false;
    if (action == LEFT || action == RIGHT) {
		reward = state.object_position != action ? 10 : -100;
		state.object_position = random_num <= 0.5 ? LEFT : RIGHT;
	} else {
		reward = -1;
	}
	return terminal;


} 

bool Search::Step(State& s, double random_num, ACT_TYPE action, double& reward,
	OBS_TYPE& obs) const {
    //std::cout<<"step"<<obs<<std::endl;
	SearchState& state = static_cast<SearchState&>(s);
	bool terminal = false;

	if (action == LEFT || action == RIGHT) {
		reward = state.object_position != action ? 10 : -100;
		state.object_position = random_num <= 0.5 ? LEFT : RIGHT;
		obs = 2; // can use arbitary observation
	} else {
		reward = -1;
		if (random_num <= 1 - NOISE)
			obs = state.object_position;
		else
			obs = (LEFT + RIGHT - state.object_position);
	}
	return terminal;
}

int Search::NumStates() const {
	return 2;
}

int Search::NumActions() const {
	return 3;
}


double Search::Reward(const State& state,  ACT_TYPE a) const
{
    std::cout<<"search reward function"<<std::endl;
    const SearchState* s= static_cast<const SearchState*>(&state);
    double reward = 0;
    if (a== LEFT || a== RIGHT) {
        reward =s->object_position!= a? 10 : -100;
    } else {
        reward = -1;
    }
    return reward;
}


double Search::Reward(int s, ACT_TYPE a) const
{
    //std::cout<<"reward function"<<std::endl;
    //ROS_INFO("reward");
    double reward = 0;
    if (a== LEFT || a== RIGHT) {
        reward =s!= a? 10 : -100;
    } else {
        reward = -1;
	}
    return reward;
}



double Search::ObsProb(OBS_TYPE obs, const State& s, ACT_TYPE a) const {
	const SearchState& state = static_cast<const SearchState&>(s);

    //std::cout<<"ObsProb"<<std::endl;
	if (a != LISTEN)
        return (obs==2);
		//return 0.0;

	return state.object_position == obs ? (1 - NOISE) : NOISE;
}

State* Search::CreateStartState(string type) const {
	return new SearchState(Random::RANDOM.NextInt(2));
}

Belief* Search::InitialBelief(const State* start, string type) const {
    std::cout<<"search_belief"<<std::endl;
	vector<State*> particles;
	SearchState* left = static_cast<SearchState*>(Allocate(-1, 0.5));
	left->object_position = LEFT;
	particles.push_back(left);
	SearchState* right = static_cast<SearchState*>(Allocate(-1, 0.5));
	right->object_position = RIGHT;
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
	} else if (name == "LEFT") {
		bound = new BlindPolicy(this, LEFT,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "RIGHT") {
		bound = new BlindPolicy(this, RIGHT,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "LISTEN") {
		bound = new BlindPolicy(this, LISTEN,
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
	out << (obs == LEFT ? "LEFT" : "RIGHT") << endl;
}

void Search::PrintAction(ACT_TYPE action, ostream& out) const {
	if (action == LEFT) {
		out << "Open left" << endl;
	} else if (action == RIGHT) {
		out << "Open right" << endl;
	} else {
		out << "Listen" << endl;
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
