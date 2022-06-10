#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>

#define PRECISION 6
#define epsilon 10E-6
#define SEED 3256+2585

using namespace std;

enum HA {
    ACC, // Constant acceleration
    DEC, // Constant deceleration
    CON  // No acceleration
};

struct Obs {
    double pos; // position
    double vel; // velocity
};

struct LA {
    double acc; // target acceleration
};

// Random error distributions
random_device rd;
default_random_engine gen(SEED);

class Robot {

    private:
    public:

    double accMax; // Maximum constant acceleration
    double decMax; // Maximum constant deceleration 
    double vMax;     // Maximum velocity

    double target; // Target distance


    normal_distribution<double> vErrDistr;        // Velocity error distribution
    double haProbCorrect;                                         // Probability of transitioning to the correct high-level action
    int model;                                                                // Use hand-written ASP (0), LDIPS-generated ASP without error (1), LDIPS-generated ASP with error (2), probabilstic ASP (3)

    Robot(double _accMax, double _decMax, double _vMax, double _target, normal_distribution<double> _vErrDistr, double _haProbCorrect, int _model){
        accMax = _accMax;
        decMax = _decMax;
        vMax = _vMax;
        target = _target;
        vErrDistr = _vErrDistr;
        haProbCorrect = _haProbCorrect;
        model = _model;
    }

    double a = 0; // acceleration
    double v = 0; // velocity
    double x = 0; // displacement

    HA ha = CON; // Initial high-level action

    // Return the distance this robot would travel before stopping if it began decelerating immediately
    double DistTraveled(double v, double dec){
        return - v * v / (2 * dec);
    }

    double logistic(double midpoint, double steepness, double input){
        return 1.0 / (1.0 + exp(-steepness * (input - midpoint)));
    }

    bool sampleDiscrete(double probTrue){
        double rv = ((double) rand())/RAND_MAX;
        return rv <= probTrue;
    }

    /*
     * This is a hand-crafted action-selection policy.
     */
    void changeHA_Hand(){
        double xToTarget = target - x;                                                                                // distance to the target

        bool cond1 = v - vMax >= 0;                                                                                     // is at max velocity (can no longer accelerate)
        bool cond2 = xToTarget - DistTraveled(v, decMax) < epsilon;                     // needs to decelerate or else it will pass target

        if(cond2){
            ha = DEC;
        }
        if(cond1 && !cond2){
            ha = CON;
        }
        if(!cond1 && !cond2){
            ha = ACC;
        }
    }

    /*
     * This is a probabilistic hand-crafted action-selection policy.
     */
    void changeHA_Hand_prob(){
        double xToTarget = target - x;                                                    // distance to the target
        bool cond1 = vMax - v < 0;                                                            // is at max velocity (can no longer accelerate)
        bool cond2 = xToTarget - DistTraveled(v, decMax) < 0;     // needs to decelerate or else it will pass target

        bool cond1smooth = sampleDiscrete(logistic(vMax*0.1, -50.0/vMax, vMax-v));
        bool cond2smooth = sampleDiscrete(logistic(target*0.1, -50.0/target, xToTarget - DistTraveled(v, decMax)));

        if(cond2smooth){
            ha = DEC;
        }
        if(cond1smooth && !cond2smooth){
            ha = CON;
        }
        if(!cond1smooth && !cond2smooth){
            ha = ACC;
        }
    }

    /*
     * This is an action-selection policy generated by LDIPS, without error.
     */
    void changeHA_LDIPS(){
        // Copy paste below
        if(ha == ACC && DistTraveled(v, decMax) + x - target >= -2.320007)
            ha = DEC;
        else if(ha == CON && DistTraveled(v, decMax) - DistTraveled(vMax, decMax) >= -150.000000 && DistTraveled(vMax, decMax) + x - target >= -0.095000)
            ha = DEC;
        else if(ha == ACC && v - vMax >= -0.025000 && x - target >= -499.975006)
            ha = CON;
        else if(ha == CON && vMax - v >= 0.000000)
            ha = ACC;
        else if(ha == DEC && v >= -1.000000)
            ha = DEC;
        else if(ha == ACC && x >= -0.997500)
            ha = ACC;
        else if(ha == CON && v >= 0.000000 && DistTraveled(vMax, decMax) - x >= -128.399994)
            ha = CON;
    }

    /*
     * This is an action-selection policy generated by LDIPS, with error.
     */
    void changeHA_LDIPS_error(){
        // Copy paste below
        if(ha == CON && DistTraveled(v, decMax) - DistTraveled(vMax, decMax) >= 9.714069)
            ha = CON;
        else if(ha == DEC && DistTraveled(v, decMax) - target >= 11.458965)
            ha = CON;
        else if(ha == CON && x + x + x - target >= 35.615170)
            ha = DEC;
        else if(ha == ACC && DistTraveled(v, decMax) + target >= 535.545532)
            ha = CON;
        else if(ha == CON)
            ha = ACC;
        else if(ha == ACC && x - target + DistTraveled(v, decMax) >= -0.138184)
            ha = DEC;
        else if(ha == DEC && DistTraveled(v, decMax) - x - x >= -49.242615)
            ha = ACC;
        else if(ha == DEC)
            ha = DEC;
        else if(ha == ACC)
            ha = ACC;
    }

    /*
     * Randomly transitions to an incorrect high-level action with specified probability
     */
    void putErrorIntoHA(int prevHA){
        double r = ((double) rand()) /RAND_MAX;
        int haDif = r < haProbCorrect ? 0 :
                                (r < 1-(1-haProbCorrect)/2) ? 1 : 2;
        ha = static_cast<HA>((ha + haDif)%3);
    }

    /*
     * Transition robot high-level action based on current global state. Runs once per time step.
     * Uses a provided method as the ASP.
     */
    void changeHA(void (*ASP) ()){
        int prevHA = ha;
        ASP();
        putErrorIntoHA(prevHA);
    }

    /*
     * Transition robot high-level action based on current global state. Runs once per time step
     */
    void changeHA(){
        int prevHA = ha;

        if(model == 0) {
        changeHA_Hand();
        } else if(model == 1){
        changeHA_LDIPS();
        } else if(model == 2){
        changeHA_LDIPS_error();
        } else if(model == 3){
        changeHA_Hand_prob();
        } else{
        exit(1);
        }

        putErrorIntoHA(prevHA);
    }

    /*
     * Given a current high-level action, apply a motor controller and update observed state. Runs once per time step
     */
    void updatePhysics(double t_step){
        double vPrev = v;
        double xPrev = x;

        // Induce some error
        double vErr = vErrDistr(gen);

        // Select some action (acceleration)
        a = ha == DEC ? decMax :
                            (ha == ACC ? accMax : 0);

        // Update velocity and displacement accordingly
        v = vPrev + a * t_step;

        if(abs(v) >= epsilon){ // v != 0
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

        x = xPrev + (v + vPrev)/2 * t_step;
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
        ha = CON;
    }
};