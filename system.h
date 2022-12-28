#pragma once

#include "robot.h"
#include "utils.h"

using namespace std;
using namespace SETTINGS;

// ----- Markov System Definitions --------------------------------
struct Obs {
    map<string, double> table;

    Obs () {
        for(Var each: Obs_vars){
            table[each.name_] = 0;
        }
    }

    Obs (map<string, double> _table){
        table = _table;

        for(Var each: Obs_vars){
            if(!contains(each.name_))
                table[each.name_] = 0;
        }
    }

    bool contains(string s){
        return table.count(s) > 0;
    }

    double get(string s) {
        return table[s];
    }

    void put(string s, double d){
        table[s] = d;
    }
};

struct State {
    HA ha;
    Obs obs;

    State () : ha(), obs() {}

    State (map<string, double> _table) : ha(), obs(_table) {}

    State (HA _ha, Obs _obs) : ha(_ha), obs(_obs) {}

    bool contains(string s){
        return obs.table.count(s) > 0;
    }

    double get(string s) {
        return obs.table[s];
    }

    void put(string s, double d){
        obs.table[s] = d;
    }

    string to_string() {
        string s = "HA: " + print(ha);
        for(Var each: Obs_vars) {
            s += ", " + each.name_ + " : " + std::to_string(get(each.name_));
        }
        return s + "\n";
    }
};

class Robot;

typedef HA asp(State);
typedef Obs motor(State, bool);
typedef Obs phys(State, double);

// -----------------------------------------------------------------------------
// ----- Robot Class------------------------------------------------------------
// -----------------------------------------------------------------------------
class Robot {
public:

    State state;

    Robot() : state() {}

    Robot(State _state) : state(_state) {};

    void runASP(asp* ASP){
        state.ha = ASP(state);
    }

    void updateLA(motor* motor_model, bool error=true){
        state.obs = motor_model(state, error);
    }

    void updateObs(phys* phys_model, double t_step = T_STEP){
        state.obs = phys_model(state, t_step);
    }

    // Reset robot
    void reset(){
        state = State();
    }
};

// -----------------------------------------------------------------------------
// ----- Trajectory ------------------------------------------------------------
// -----------------------------------------------------------------------------
struct Trajectory {
    vector<State> traj;

    Trajectory() : traj() {}

    void append(State s) {
        traj.push_back(s);
    }

    State get(int t) {
        return traj[t];
    }

    void set(int t, State s){
        traj[t] = s;
    }

    void set(int t, HA ha){
        traj[t].ha = ha;
    }

    void set(int t, Obs obs){
        traj[t].obs = obs;
    }

    int size() {
        return traj.size();
    }

    string to_string() {
        string s = "";
        for(uint32_t t = 0; t < traj.size() - 1; t++) {
            s += std::to_string(traj[t].ha) + ",";
        }
        s += std::to_string(traj[traj.size() - 1].ha) + "\n";
        return s;
    }
};
