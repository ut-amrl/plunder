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

double epsilon = 10E-14;

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

    if(st == CON){
      if(!cond1 && !cond2){
        st = ACC;
      } else if (cond2 && !cond3){
        st = DEC;
      } else {
        st = CON;
      }
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

  void updatePhysics(){
    double vPrev = v;
    double xPrev = x;

    double vErr = vErrDistr(gen);

    a = st == DEC ? decMax :
              (st == ACC ? accMax : 0);
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

void moveRobotAndWriteJson(ofstream& jsonFile, bool isFirst, Robot& r){
  string prevStateStr = r.st == 0 ? "ACC" : 
                        (r.st == 1 ? "DEC" : "CON");
  r.updatePhysics();
  r.changeState();

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
  
  vector<Robot> robots;
  robots.push_back(Robot(4, -4, 15, 100));
  robots.push_back(Robot(2, -2, 15, 100));
  robots.push_back(Robot(3, -1, 12, 200));
  robots.push_back(Robot(6, -2, 30, 50));
  robots.push_back(Robot(2, -10, 10, 80));
  
  if(genJson){
    cout << "generating json\n";
    ofstream jsonFile;
    jsonFile.open("data.json");
    jsonFile << "[";
    for(int i=0; i<robots.size(); i++){
      for(double t=0; t<T_TOT/T_STEP; t++){
        moveRobotAndWriteJson(jsonFile, (t==0 && i==0), robots[i]);
      }
    }
    jsonFile << "]";
    jsonFile.close();
  }

  for(int i = 0; i < robots.size(); i++){
    robots[i].reset();
  }

  if(genCsv){
    cout << "generating csv\n";
    ofstream csvFile;
    csvFile.open("data.csv");
    csvFile << "time, x, v" << "\n";
    for(double t=0; t<T_TOT/T_STEP; t++){
      for(int i=0; i<robots.size(); i++){
        robots[i].updatePhysics();
        robots[i].changeState();
        csvFile << t <<  ", " << robots[i].x << ", " << robots[i].v << "\n";
      }
    }
    csvFile.close();
  }



  return 0;
}
