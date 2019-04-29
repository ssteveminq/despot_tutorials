//#include <despot/simple_tui.h>
#include <despot/planner.h>
#include <tiger.h>
#include <tiger_world.h>
#include <despot/util/seeds.h>

using namespace despot;

//class TUI: public SimpleTUI {

class MyPlanner: public Planner {
public:
  MyPlanner() {
  }
 
  DSPOMDP* InitializeModel(option::Option* options) {
      //DSPOMDP* model = !options[E_PARAMS_FILE] ?
    //new Tiger() : new Tiger(options[E_PARAMS_FILE].arg);
    DSPOMDP* model = new Tiger();
    return model;
  }
  
  World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options)
  {
      //std::cout<<"world initialized"<<std::endl;
      //Create a custom world as defined and implemented by the user
      //TigerWorld* world = new TigerWorld();
       //return InitializePOMDPWorld(world_type,model, options);
      TigerWorld* world = new TigerWorld(model, Seeds::Next());
      //Establish connection with external system
      world->Connect();
      //Initialize the state of the external system
      world->Initialize();
      //static_cast<Tiger*>(model)->NoiseSigma(TigerWorld::noise_sigma_);
      //Inform despot the type of world
      world_type = "simulator";

      std::cout<<"tigerworld initialized done"<<std::endl;
      return world; 
  }

  void InitializeDefaultParameters() {

    Globals::config.pruning_constant = 0.01;
  }

  std::string ChooseSolver(){
	  return "DESPOT";
  }

  void PlanningLoop(Solver*& solver, World* world, Logger* logger) {
    for (int i = 0; i < Globals::config.sim_len; i++) {
      bool terminal = RunStep(solver, world, logger);
      if (terminal)
        break;
    }
  }

  //bool RunStep(Solver* solver, World* world, Logger* logger) {
    //logger->CheckTargetTime();

    //double step_start_t = get_time_second();

    //double start_t = get_time_second();
    //ACT_TYPE action = solver->Search().action;
    //double end_t = get_time_second();
    //double search_time = (end_t - start_t);
    //logi << "[Custom RunStep] Time spent in " << typeid(*solver).name()
        //<< "::Search(): " << search_time << endl;

    //OBS_TYPE obs;
    //start_t = get_time_second();
    //bool terminal = world->ExecuteAction(action, obs);
    //end_t = get_time_second();
    //double execute_time = (end_t - start_t);
    //logi << "[Custom RunStep] Time spent in ExecuteAction(): " << execute_time << endl;

    //start_t = get_time_second();
    //solver->BeliefUpdate(action, obs);
    //end_t = get_time_second();
    //double update_time = (end_t - start_t);
    //logi << "[Custom RunStep] Time spent in Update(): " << update_time << endl;

    //return logger->SummarizeStep(step_++, round_, terminal, action, obs,
        //step_start_t);
  //}


};

int main(int argc, char* argv[]) {
  return MyPlanner().RunPlanning(argc, argv);
  //return MyPlanner().RunEvaluation(argc, argv);
}
