#include <iostream>
#include <random>
#include <fstream>

using namespace std;

static const bool genCsv = true;
static const bool genJson = true;

static const double T_STEP = .05;
static const double T_TOT = 15;

static const double stoppingDistanceMean = 10.0;
static const double stoppingDistanceStdDev = 0.0;
static const double vErrMean = 0.0;
static const double vErrStdDev = 0.0;


enum State {
  ACC,
  DEC,
  CON
};

random_device rd;
default_random_engine gen(rd());
normal_distribution<double> stoppingDistanceDistr(stoppingDistanceMean, stoppingDistanceStdDev);
normal_distribution<double> vErrDistr(vErrMean, vErrStdDev);

// double epsilon = 10E-14;

class Robot {

  private:
  public:

  double accMax;
  double decMax;
  double vMax;

  double target;
  double stoppingDistance;

  Robot(double _accMax, double _decMax, double _vMax, double _target){
    accMax = _accMax;
    decMax = _decMax;
    vMax = _vMax;
    target = _target;
    stoppingDistance = stoppingDistanceDistr(gen);
  }

  double a = 0;
  double v = 0;
  double x = 0;

  State st = CON; // Initial state


  void changeState(){
    double xToTarget = target - x - stoppingDistance;

    bool cond1 = v - vMax >= 0;                                   // is at max velocity
    bool cond2 = xToTarget + v * v / (2 * decMax) < 0;            // needs to decelerate or else it will pass target
    bool cond3 = v <= 0;                                          // is at min velocity

    if(cond2 && !cond3){
      st = DEC;
    }
    if((cond1 && !cond2) || cond3){
      st = CON;
    }
    if(!cond1 && !cond2){
      st = ACC;
    }
  }

  void updatePhysics(){
    double vPrev = v;
    double xPrev = x;

    double vErr = vErrDistr(gen);

    a = st == DEC ? decMax :
              (st == ACC ? accMax : 0);
    v = vPrev + a * T_STEP;
    bool vIsZero = v <= 0;
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

void writeJson(ofstream& jsonFile, bool isFirst, Robot& r){
  string prevStateStr = r.st == 0 ? "ACC" : 
                        (r.st == 1 ? "DEC" : "CON");
  r.changeState();
  r.updatePhysics();

  string curStateStr = r.st == 0 ? "ACC" : 
                    (r.st == 1 ? "DEC" : "CON");
  if(!isFirst) jsonFile << ",";
  jsonFile << R"({"x":{"dim":[1,0,0],"type":"NUM","name":"x","value":)";
  jsonFile << r.x;
  jsonFile << R"(},"v":{"dim":[1,-1,0],"type":"NUM","name":"v","value":)";
  jsonFile << r.v;
  jsonFile << R"(},"target":{"dim":[1,0,0],"type":"NUM","name":"target","value":)";
  jsonFile << r.target;
  jsonFile << R"(},"zeroVel":{"dim":[1,-1,0],"type":"NUM","name":"zeroVel","value":)";
  jsonFile << 0;
  jsonFile << R"(},"vMax":{"dim":[1,-1,0],"type":"NUM","name":"vMax","value":)";
  jsonFile << r.vMax;
  jsonFile << R"(},"decMax":{"dim":[1,-2,0],"type":"NUM","name":"decMax","value":)";
  jsonFile << r.decMax;
  jsonFile << R"(},"start":{"dim":[0,0,0],"type":"STATE","name":"output","value":")";
  jsonFile << prevStateStr;
  jsonFile << R"("},"output":{"dim":[0,0,0],"type":"STATE","name":"output","value":")";
  jsonFile << curStateStr;
  jsonFile << R"("}})" << endl;
}



int main() {
  
  Robot r1 = Robot(4, -4, 15, 100);
  Robot r2 = Robot(2, -2, 15, 100);
  
  if(genJson){
    cout << "generating json\n";
    ofstream jsonFile;
    jsonFile.open("data.json");
    jsonFile << "[";
    for(double t=0; t<T_TOT/T_STEP; t++){
      writeJson(jsonFile, t==0, r1);
    }
    for(double t=0; t<T_TOT/T_STEP; t++){
      writeJson(jsonFile, false, r2);
    }
    jsonFile << "]";
    jsonFile.close();
  }

  r1.reset();
  r2.reset();

  if(genCsv){
    cout << "generating csv\n";
    ofstream csvFile;
    csvFile.open("data.csv");
    csvFile << "time, x, v" << "\n";
    for(double t=0; t<T_TOT/T_STEP; t++){
      r1.changeState();
      r1.updatePhysics();
      r2.changeState();
      r2.updatePhysics();
      csvFile << t << ", " << r1.x << ", " << r1.v << "\n";
      csvFile << t << ", " << r2.x << ", " << r2.v << "\n";
    }
    csvFile.close();
  }



  return 0;
}
