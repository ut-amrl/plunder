#include <iostream>
#include <random>
#include <fstream>

using namespace std;


bool genCsv = true;
bool genJson = true;


enum State {
  ACC,
  DEC,
  CON
};

double T_STEP = .02;
double T_TOT = 15;

std::random_device rd;
std::default_random_engine gen(rd());

double stoppingDistanceMean = 10.0;
double stoppingDistanceStdDev = 2.0;
std::normal_distribution<double> stoppingDistanceDistr(stoppingDistanceMean, stoppingDistanceStdDev);
double xErrMean = 0.0;
double xErrStdDev = 1.0;
std::normal_distribution<double> xErrDistr(xErrMean, xErrStdDev);

double epsilon = 10E-14;

class Robot {


  private:
  public:

  double accMax = 4;
  double decMax = -4;
  double vMax = 15;

  double target = 100;
  double stoppingDistance = stoppingDistanceDistr(gen);

  double a = 0;
  double v = 0;
  double x = 0;

  State st = CON;




  void changeState(){
    double xToTarget = target - x - stoppingDistance;

    bool cond1 = v >= vMax;                           // is at max velocity
    bool cond2 = xToTarget < (-v * v)/(2 * decMax);   // needs to decelerate or else it will pass target
    bool cond3 = v <= epsilon;                        // is at min velocity

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

    double xErr = xErrDistr(gen);
    cout << xErr << " ";

    a = st == DEC ? decMax :
              (st == ACC ? accMax : 0);
    v = vPrev + a * T_STEP;
    bool vIsZero = abs(v) <= epsilon;
    x = xPrev + (v + vPrev)/2 * T_STEP + (vIsZero ? 0 : xErr);
  }

};



int main() {
  


  if(genCsv){
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
