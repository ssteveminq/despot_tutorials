#ifndef SEARCH_H
#define SEARCH_H

#include <despot/interface/pomdp.h>
#include <despot/util/random.h>

namespace despot {

/* =============================================================================
 * SearchState class
 * =============================================================================*/

class SearchState: public State {
public:
	int object_position;

	SearchState();

	SearchState(int position);

	std::string text() const;
};

/* =============================================================================
 * Search class
 * =============================================================================*/

class Search: public DSPOMDP {
private:
	mutable MemoryPool<SearchState> memory_pool_;

//protected:
	//std::vector<OBS_TYPE> obs_;
	//std::vector<RegDemoState*> states_;

	//std::vector<std::vector<std::vector<State> > > transition_probabilities_; //state, action, [state, weight]
	//mutable std::vector<ACT_TYPE> default_action_;


public:

    enum {
        //state
        VISIBLE=1, OCCLUSION=2, MISSING=3
    };

    enum {
        //observations
        TARGET =1, OCC_NOTARGET=2, MISS_NOTARGET=3
    };

	static const ACT_TYPE SEARCH, MOVE, WAIT;
	static const double NOISE;

	Search();
	Search(std::string params_file);

    bool Step_world( State& s, double random_num,  ACT_TYPE action, double& reward, OBS_TYPE obs) const; 

	bool Step(State& s, double random_num, ACT_TYPE action, double& reward,
		OBS_TYPE& obs) const;
	int NumStates() const;
	int NumActions() const;
	double ObsProb(OBS_TYPE obs, const State& s, ACT_TYPE a) const;

    double Reward(const State& s,  ACT_TYPE a) const;
	double Reward(int s,  ACT_TYPE a) const;


	State* CreateStartState(std::string type) const;
	Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

	inline double GetMaxReward() const {
		return 50;
	}

	inline ValuedAction GetBestAction() const {
		return ValuedAction(WAIT, -1);
	}
	ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;

	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
	void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const;

	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;
};

} // namespace despot

#endif
