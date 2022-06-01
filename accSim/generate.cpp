#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>

#define PRECISION 6
#define epsilon 10E-6
#define SEED 3256+2585

using namespace std;

// Parameters
static const bool genCsv = true;        // generate CSV trace file
static const bool genJson = true;       // generate JSON trace file
static const int useModel = 2;          // use hand-written ASP (0), LDIPS-generated ASP without error (1), or LDIPS-generated ASP with error (2)
static const int robotTestSet = 2;      // which robot test set to use (1-2)
static const bool velocityError = false; // apply error to velocity
static const bool actionError = false;   // apply error to state transitions

// Configuration & Global variables
static const double T_STEP = .1;  // time step
static const double T_TOT = 15;   // total time per simulated scenario

// Error distribution parameters
static const double vErrMean = 0.0;     // velocity error distribution
static const double vErrStdDev = 0.1;

// Action error probabilities
static const double stProbCorrect = 0.96;

enum State {
  ACC, // Constant acceleration
  DEC, // Constant deceleration
  CON  // No acceleration
};

// Diagnostics
vector<vector<int>> stateTransitionPosCount;
vector<vector<int>> stateTransitionNegCount;

// Random error distributions
random_device rd;
default_random_engine gen(SEED);
normal_distribution<double> vErrDistr(vErrMean, vErrStdDev);

class Robot {

  private:
  public:

  double accMax; // Maximum constant acceleration
  double decMax; // Maximum constant deceleration 
  double vMax;   // Maximum velocity

  double target; // Target distance

  Robot(double _accMax, double _decMax, double _vMax, double _target){
    accMax = _accMax;
    decMax = _decMax;
    vMax = _vMax;
    target = _target;
  }

  double a = 0; // acceleration
  double v = 0; // velocity
  double x = 0; // displacement

  State st = CON; // Initial state

  // Return the distance this robot would travel before stopping if it began decelerating immediately
  double DistTraveled(double v, double dec){
    return - v * v / (2 * dec);
  }

  /*
   * This is a hand-crafted action-selection policy.
   */
  void changeState_Hand(){
    double xToTarget = target - x;                                        // distance to the target

    bool cond1 = v - vMax >= 0;                                           // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget - DistTraveled(v, decMax) < epsilon;           // needs to decelerate or else it will pass target

    if(cond2){
      st = DEC;
    }
    if(cond1 && !cond2){
      st = CON;
    }
    if(!cond1 && !cond2){
      st = ACC;
    }
  }

  /*
   * This is an action-selection policy generated by LDIPS, without error.
   */
  void changeState_LDIPS(){
    // Copy paste below
    if(st == ACC && DistTraveled(v, decMax) + x - target >= -2.320007)
      st = DEC;
    else if(st == CON && DistTraveled(v, decMax) - DistTraveled(vMax, decMax) >= -150.000000 && DistTraveled(vMax, decMax) + x - target >= -0.095000)
      st = DEC;
    else if(st == ACC && v - vMax >= -0.025000 && x - target >= -499.975006)
      st = CON;
    else if(st == CON && vMax - v >= 0.000000)
      st = ACC;
    else if(st == DEC && v >= -1.000000)
      st = DEC;
    else if(st == ACC && x >= -0.997500)
      st = ACC;
    else if(st == CON && v >= 0.000000 && DistTraveled(vMax, decMax) - x >= -128.399994)
      st = CON;
  }

  /*
   * This is an action-selection policy generated by LDIPS, with error.
   */
  void changeState_LDIPS_error(){
    // Copy paste below
    if(st == DEC && 0 - v >= 0.000000)
      st = CON;
    else if(st == CON && 0 - v >= -4.068562)
      st = ACC;
    else if(st == CON && DistTraveled(v, decMax) + x - target >= -0.672409)
      st = DEC;
    else if(st == ACC && x >= 56.625511)
      st = DEC;
    else if(st == ACC && v - vMax >= -0.042427)
      st = CON;
    else if(st == DEC && target - x - DistTraveled(v, decMax) >= -0.967556)
      st = ACC;
    else if(st == CON)
      st = CON;
    else if(st == DEC)
      st = DEC;
    else if(st == ACC)
      st = ACC;
  }

  /*
   * Randomly transitions to an incorrect state with specified probability
   */
  void putErrorIntoState(int prevSt){
    double r = ((double) rand()) /RAND_MAX;
    int stDif = r < stProbCorrect ? 0 :
                (r < 1-(1-stProbCorrect)/2) ? 1 : 2;
    st = static_cast<State>((st + stDif)%3);
    stateTransitionPosCount[prevSt][st] += (stDif == 0);
    stateTransitionNegCount[prevSt][st] += (stDif != 0);
  }

  /*
   * Transition robot state based on current global state. Runs once per time step
   */
  void changeState(){
    int prevSt = st;

    if(useModel == 0) {
      changeState_Hand();
    } else if(useModel == 1){
      changeState_LDIPS();
    } else {
      changeState_LDIPS_error();
    }

    if(actionError){
      putErrorIntoState(prevSt);
    }
  }

  /*
   * Given a current state, apply an action controller and update kinematic variables. Runs once per time step
   */
  void updatePhysics(){
    double vPrev = v;
    double xPrev = x;

    // Induce some error
    double vErr = vErrDistr(gen);

    // Select some action (acceleration)
    a = st == DEC ? decMax :
              (st == ACC ? accMax : 0);

    // Update velocity and displacement accordingly
    v = vPrev + a * T_STEP;

    if(velocityError && abs(v) >= epsilon){ // v != 0
      v += vErr;
    }
    
    if(v < epsilon){ // Round to 0
      v = 0;
    }

    if(abs(v - vMax) < epsilon){ // Round to vMax
      v = vMax;
    }

    if(abs(x - target) < epsilon){ // Round to target
      x = target;
    }

    x = xPrev + (v + vPrev)/2 * T_STEP;
  }

  /*
   * Robot has reached target and is at rest. End simulation.
   */
  bool finished(){
    return v < epsilon && x >= target - epsilon;
  }

  void reset(){
    a = 0;
    v = 0;
    x = 0;
    st = CON;
  }
};

int main() {
  
  // Initialization
  stateTransitionPosCount.push_back(vector<int>(3)); stateTransitionPosCount.push_back(vector<int>(3)); stateTransitionPosCount.push_back(vector<int>(3));
  stateTransitionNegCount.push_back(vector<int>(3)); stateTransitionNegCount.push_back(vector<int>(3)); stateTransitionNegCount.push_back(vector<int>(3));

  // Create some arbitrary robots
  vector<Robot> robots;

  if(robotTestSet == 1){
    robots.push_back(Robot(6, -5, 15, 150));
    robots.push_back(Robot(3, -3, 30, 100));
    robots.push_back(Robot(5, -2, 12, 200));
    robots.push_back(Robot(8, -3, 30, 50));
    robots.push_back(Robot(1.5, -2, 3, 30));
    robots.push_back(Robot(3, -2, 4, 20));
    robots.push_back(Robot(0.5, -1, 2, 15));
    robots.push_back(Robot(1.5, -2, 40, 300));
    robots.push_back(Robot(2, -2, 100, 300));
    robots.push_back(Robot(5, -1, 25, 200));
    robots.push_back(Robot(10, -10, 25, 500));
  } else if(robotTestSet == 2){
    robots.push_back(Robot(5, -2, 15, 80));
    robots.push_back(Robot(3, -3, 4, 80));
    robots.push_back(Robot(1.5, -4, 50, 80));
    robots.push_back(Robot(8, -6, 20, 80));
    robots.push_back(Robot(4, -5, 100, 80));
  }
  
  // Setup output
  ofstream jsonFile;
  ofstream csvFile;
  if(genJson){
    cout << "generating json\n";
    jsonFile << fixed << setprecision(PRECISION);
    jsonFile.open("data.json");
    jsonFile << "[";
  }
  if(genCsv){
    cout << "generating csv\n";
    csvFile << fixed << setprecision(PRECISION);
    csvFile.open("data.csv");
    csvFile << "time, x, v" << "\n";
  }

  // Run simulations and generate json/csv files
  bool first = true;
  for(int i=0; i<robots.size(); i++){
    for(double t=0; t<T_TOT/T_STEP; t++){

      // Run simulation
      string prevStateStr = robots[i].st == 0 ? "ACC" : 
                        (robots[i].st == 1 ? "DEC" : "CON");

      robots[i].updatePhysics();
      robots[i].changeState();

      string curStateStr = robots[i].st == 0 ? "ACC" : 
                    (robots[i].st == 1 ? "DEC" : "CON");

      // Print trace
      if(genCsv) {
        csvFile << t << ", " << robots[i].x << ", " << robots[i].v << "\n";
      }

      if(genJson){
        if(first) first = false;
        else jsonFile << ",";
        jsonFile << R"({"x":{"dim":[1,0,0],"type":"NUM","name":"x","value":)";
        jsonFile << robots[i].x;
        jsonFile << R"(},"v":{"dim":[1,-1,0],"type":"NUM","name":"v","value":)";
        jsonFile << robots[i].v;
        jsonFile << R"(},"target":{"dim":[1,0,0],"type":"NUM","name":"target","value":)";
        jsonFile << robots[i].target;
        jsonFile << R"(},"zeroVel":{"dim":[1,-1,0],"type":"NUM","name":"zeroVel","value":)";
        jsonFile << 0;
        jsonFile << R"(},"vMax":{"dim":[1,-1,0],"type":"NUM","name":"vMax","value":)";
        jsonFile << robots[i].vMax;
        jsonFile << R"(},"decMax":{"dim":[1,-2,0],"type":"NUM","name":"decMax","value":)";
        jsonFile << robots[i].decMax;
        jsonFile << R"(},"start":{"dim":[0,0,0],"type":"STATE","name":"output","value":")";
        jsonFile << prevStateStr;
        jsonFile << R"("},"output":{"dim":[0,0,0],"type":"STATE","name":"output","value":")";
        jsonFile << curStateStr;
        jsonFile << R"("}})" << endl;
      }

      if(robots[i].finished()){
        break;
      }
    }
  }

  if(genJson){
    jsonFile << "]";
    jsonFile.close();
  }
  if(genCsv){
    csvFile.close();
  }

  for(int i = 0; i < stateTransitionPosCount.size(); i++){
    for(int j = 0; j < stateTransitionPosCount[i].size(); j++){
      cout << "Correct: " << stateTransitionPosCount[i][j] << " | Incorrect: " << stateTransitionNegCount[i][j] << "\n";
    }
    cout << endl;
  }

  return 0;
}

// ./bin/ldips-l3 -lib_file ops/test_library.json -ex_file examples/data.json -min_accuracy 0.5 -debug true window_size 0