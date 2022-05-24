#include <iostream>
#include <random>
#include <fstream>

using namespace std;

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
  
  Robot r = Robot();
  ofstream file;

  file.open("data.csv");
  file << "time, x, v" << "\n";
  for(double t=0; t<T_TOT/T_STEP; t++){
    r.changeState();
    r.updatePhysics();
    file << t << ", " << r.x << ", " << r.v << "\n";
  }
  file.close();

  return 0;
}
