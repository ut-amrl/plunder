#include <iostream>
#include <random>
#include <fstream>

using namespace std;


bool genCsv = true;
bool genJson = true;

double T_STEP = .05;
double T_TOT = 15;
double stoppingDistanceMean = 10.0;
double stoppingDistanceStdDev = 0.0;
double vErrMean = 0.0;
double vErrStdDev = 0.2;


enum State {
  ACC,
  DEC,
  CON
};

std::random_device rd;
std::default_random_engine gen(rd());
std::normal_distribution<double> stoppingDistanceDistr(stoppingDistanceMean, stoppingDistanceStdDev);
std::normal_distribution<double> vErrDistr(vErrMean, vErrStdDev);

double epsilon = 10E-14;

class Robot {


  private:
  public:

  double accMax = 4;
  double decMax = -4;
  double vMax = 15;

  double target = 100;
  double stoppingDistance = stoppingDistanceDistr(gen);

  Robot(){}

  Robot(double _accMax, double _decMax, double _vMax, double _target){
    accMax = _accMax;
    decMax = _decMax;
    vMax = _vMax;
    target = _target;
  }

  double a = 0;
  double v = 0;
  double x = 0;

  State st = CON;


  void changeState(){
    double xToTarget = target - x - stoppingDistance;

    bool cond1 = v >= vMax;                                   // is at max velocity
    // synthesis: v - vMax > 0
    bool cond2 = xToTarget < (-1.0/2.0) * v * (v / decMax);   // needs to decelerate or else it will pass target
    // synthesis: (oneHalf * (v * (v / decMax))) + (target - x) < stoppingDistanceMean
    bool cond3 = v <= epsilon;                                // is at min velocity
    // synthesis: v < 0

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
    bool vIsZero = v <= epsilon;
    v = vIsZero ? 0 : v+vErr;
    x = xPrev + (v + vPrev)/2 * T_STEP;
  }

};

void writeJson(){
  
}



int main() {
  


  if(genCsv){
    cout << "generating csv\n";
    Robot r = Robot();
    ofstream csvFile;
    csvFile.open("data.csv");
    csvFile << "time, x, v" << "\n";
    for(double t=0; t<T_TOT/T_STEP; t++){
      r.changeState();
      r.updatePhysics();
      csvFile << t << ", " << r.x << ", " << r.v << "\n";
    }
    csvFile.close();
  }


  if(genJson){
    cout << "generating json\n";
    Robot r = Robot();
    ofstream jsonFile;
    jsonFile.open("data.json");
    jsonFile << "[";
    for(double t=0; t<T_TOT/T_STEP; t++){
      string prevStateStr = r.st == 0 ? "ACC" : 
                        (r.st == 1 ? "DEC" : "CON");
      r.changeState();
      r.updatePhysics();
      string curStateStr = r.st == 0 ? "ACC" : 
                        (r.st == 1 ? "DEC" : "CON");
      if(t > 0) jsonFile << ",";
      jsonFile << R"({"x":{"dim":[1,0,0],"type":"NUM","name":"x","value":)";
      jsonFile << r.x;
      jsonFile << R"(},"v":{"dim":[1,-1,0],"type":"NUM","name":"v","value":)";
      jsonFile << r.v;
      jsonFile << R"(},"target":{"dim":[1,0,0],"type":"NUM","name":"target","value":)";
      jsonFile << r.target;
      jsonFile << R"(},"oneHalf":{"dim":[0,0,0],"type":"NUM","name":"oneHalf","value":)";
      jsonFile << 0.5;
      jsonFile << R"(},"vMax":{"dim":[1,-1,0],"type":"NUM","name":"vMax","value":)";
      jsonFile << r.vMax;
      jsonFile << R"(},"decMax":{"dim":[1,-2,0],"type":"NUM","name":"decMax","value":)";
      jsonFile << r.decMax;
      jsonFile << R"(},"start":{"dim":[0,0,0],"type":"STATE","name":"output","value":")";
      jsonFile << prevStateStr;
      jsonFile << R"("},"output":{"dim":[0,0,0],"type":"STATE","name":"output","value":")";
      jsonFile << curStateStr;
      jsonFile << R"("}})";
    }
    jsonFile << "]";
    jsonFile.close();
  }

  return 0;
}
