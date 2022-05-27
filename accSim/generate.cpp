#include <iostream>
#include <random>
#include <fstream>

#define epsilon 10E-14 // ~= 0

using namespace std;

static const bool genCsv = true;
static const bool genJson = true;

static const double T_STEP = .05; // time step
static const double T_TOT = 15; // total time per simulated scenario

// Error distribution parameters
static const double stoppingDistanceMean = 10.0; // stopping distance distribution
static const double stoppingDistanceStdDev = 0.0;
static const double vErrMean = 0.0; // velocity error distribution
static const double vErrStdDev = 0.0;

enum State {
  ACC, // Constant acceleration
  DEC, // Constant deceleration
  CON  // No acceleration
};

// Random error distributions
random_device rd;
default_random_engine gen(rd());
normal_distribution<double> stoppingDistanceDistr(stoppingDistanceMean, stoppingDistanceStdDev);
normal_distribution<double> vErrDistr(vErrMean, vErrStdDev);

class Robot {

  private:
  public:

  double accMax; // Maximum constant acceleration
  double decMax; // Maximum constant deceleration 
  double vMax;   // Maximum velocity

  double target; // Target distance
  double stoppingDistance; // Stop a certain distance before the target distance (to prevent collisions)

  Robot(double _accMax, double _decMax, double _vMax, double _target){
    accMax = _accMax;
    decMax = _decMax;
    vMax = _vMax;
    target = _target;
    stoppingDistance = stoppingDistanceDistr(gen);
  }

  double a = 0; // acceleration
  double v = 0; // velocity
  double x = 0; // displacement

  State st = CON; // Initial state

  /*
   * Transition robot state based on current global state. Runs once per time step
   */
  void changeState(){
    double xToTarget = target - x - stoppingDistance;             // distance to the target

    bool cond1 = v - vMax >= 0;                                   // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget + v * v / (2 * decMax) < 0;            // needs to decelerate or else it will pass target
    bool cond3 = v <= 0;                                          // is at min velocity (can no longer decelerate)

    if(st == CON){
      if(!cond1 && !cond2){
        st = ACC;
      } else if (cond2 && !cond3){
        st = DEC;
      } else {
        st = CON;
      }

      // Triangular
      // if(cond2){
      //   st = CON;
      // } else {
      //   st = ACC;
      // }
    } else if (st == ACC){
      if(!cond1 && !cond2){
        st = ACC;
      } else if(cond2){
        st = DEC;
      } else {
        st = CON;
      }

      // Triangular
      // if(cond2){
      //   st = DEC;
      // } else {
      //   st = ACC;
      // }
    } else if (st == DEC){
      if(cond3){
        st = CON;
      } else {
        st = DEC;
      }
    }

    // Simplified version
    // if(cond2 && !cond3){
    //   st = DEC;
    // }
    // if((cond1 && !cond2) || cond3){
    //   st = CON;
    // }
    // if(!cond1 && !cond2){
    //   st = ACC;
    // }
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
    bool vIsZero = v <= epsilon;
    v = vIsZero ? 0 : v+vErr;
    x = xPrev + (v + vPrev)/2 * T_STEP;
  }


  void reset(){
    a = 0;
    v = 0;
    x = 0;
    st = CON;
  }
};

int main() {
  
  // Create some arbitrary robots
  vector<Robot> robots;
  robots.push_back(Robot(4, -4, 15, 150));
  robots.push_back(Robot(2, -2, 15, 100));
  robots.push_back(Robot(3, -1, 12, 200));
  robots.push_back(Robot(6, -2, 30, 50));
  robots.push_back(Robot(2, -10, 10, 80));
  robots.push_back(Robot(1, -1, 3, 30));
  robots.push_back(Robot(2, -1, 4, 20));
  
  // Setup output
  ofstream jsonFile;
  ofstream csvFile;
  if(genJson){
    cout << "generating json\n";
    jsonFile.open("data.json");
    jsonFile << "[";
  }
  if(genCsv){
    cout << "generating csv\n";
    csvFile.open("data.csv");
    csvFile << "time, x, v" << "\n";
  }

  // Run simulations and generate json/csv files
  for(int i=0; i<robots.size(); i++){
    for(double t=0; t<T_TOT/T_STEP; t++){
      string prevStateStr = robots[i].st == 0 ? "ACC" : 
                        (robots[i].st == 1 ? "DEC" : "CON");

      robots[i].updatePhysics();
      robots[i].changeState();

      string curStateStr = robots[i].st == 0 ? "ACC" : 
                    (robots[i].st == 1 ? "DEC" : "CON");

      if(genCsv) {
        csvFile << t << ", "  << robots[i].x << ", " << robots[i].v << "\n";
      }

      if(genJson){
        if(t != 0 || i != 0) jsonFile << ",";

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
    }
  }

  if(genJson){
    jsonFile << "]";
    jsonFile.close();
  }
  if(genCsv){
    csvFile.close();
  }

  return 0;
}
