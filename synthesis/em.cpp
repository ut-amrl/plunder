#include "ast/ast.hpp"
#include "ast/enumeration.hpp"
#include "ast/library_functions.hpp"
#include "ast/parsing.hpp"
#include "visitors/interp_visitor.hpp"
#include "ast/synthesis.hpp"

#include "particleFilter/pf_runner.h"
#include "accSim/generate.h"

using namespace std;
using namespace AST;
using namespace SETTINGS;
using Eigen::Vector2f;
using json = nlohmann::json;

unordered_set<Var> variables;
vector<pair<string,string>> transitions;
vector<FunctionEntry> library;
vector<ast_ptr> roots;
PyObject* pFunc;

vector<float> accuracies;
vector<ast_ptr> preds;
vector<ast_ptr> gt_truth;

namespace std {
    ostream& operator<<(ostream& os, const AST::ast_ptr& ast);
}

// Convert transition to EMDIPS-compatible Example
Example dataToExample(HA ha, Obs state, Robot& robot){
    Example ex;

    ex.symbol_table_["x"] = SymEntry((float) state.pos);
    ex.symbol_table_["v"] = SymEntry((float) state.vel);
    ex.symbol_table_["decMax"] = SymEntry((float) robot.decMax);
    ex.symbol_table_["vMax"] = SymEntry((float) robot.vMax);
    ex.symbol_table_["target"] = SymEntry((float) robot.target);

    ex.start_ = SymEntry(to_string(ha));

    return ex;
}

void printExampleInfo(Example e){
    float x = e.symbol_table_["x"].GetFloat();
    float v = e.symbol_table_["v"].GetFloat();
    float decMax = e.symbol_table_["decMax"].GetFloat();
    float vMax = e.symbol_table_["vMax"].GetFloat();
    float target = e.symbol_table_["target"].GetFloat();
    string start = e.start_.GetString();
    string res = e.result_.GetString();
    float exp = v-vMax;
    cout << start << "->" << res << ", x " << x << ", v " << v << ", decMax " << decMax << ", vMax " << vMax << ", target " << target << ", exp " << exp << endl;
}

// Runs EMDIPS-generated ASP
HA emdipsASP(State state, Robot& robot){
    HA prevHA = state.ha;
    Example obsObject = dataToExample(state.ha, state.obs, robot);
    for(uint i = 0; i < transitions.size(); i++){
        if(to_string(prevHA) == transitions[i].first && transitions[i].first!=transitions[i].second){
            if(InterpretBool(preds[i], obsObject)) {
                state.ha = to_label(transitions[i].second);
                break;
            }
        }
    }

    return pointError(state.ha, POINT_ACCURACY, USE_SAFE_TRANSITIONS); // Introduce point errors - random transitions allow GT_ASP to escape local minima
}

// Initial ASP: random transitions
HA initialASP(State state, Robot& r) {
    return pointError(state.ha, POINT_ACCURACY, USE_SAFE_TRANSITIONS);
}

bool isValidExample(Example ex){
    // return (ex.start_.GetString()==("ACC") && ex.result_.GetString()==("CON")) ||
    //         (ex.start_.GetString()==("ACC") && ex.result_.GetString()==("ACC")) ||
    //         (ex.start_.GetString()==("ACC") && ex.result_.GetString()==("DEC")) ||
    //         (ex.start_.GetString()==("CON") && ex.result_.GetString()==("DEC")) ||
    //         (ex.start_.GetString()==("CON") && ex.result_.GetString()==("CON")) ||
    //         (ex.start_.GetString()==("DEC") && ex.result_.GetString()==("DEC"));
    return true;
}

void plot_pure(Trajectory& traj, asp* asp, string output_path) {

    ofstream outFile;
    outFile.open(output_path);

    for(uint32_t n = 0; n < PARTICLES_PLOTTED; n++){
        execute_pure(traj, asp);

        for(uint32_t t = 0; t < traj.T - 1; t++) {
            outFile << traj.get(t).ha << ",";
        }
        outFile << traj.get(traj.T - 1).ha << endl;

    }
    outFile.close();
}

// Expectation step
vector<vector<Example>> expectation(uint iteration, vector<Robot>& robots, vector<vector<Obs>>& dataObs, vector<vector<LA>>& dataLa, asp* asp){

    vector<vector<Example>> examples;

    cout << "Running particle filter with " << NUM_PARTICLES << " particles\n";
    cout << "Parameters: resample threshold=" << RESAMPLE_THRESHOLD << ", observation strength=" << OBS_LIKELIHOOD_STRENGTH << endl;

    double cum_log_obs = 0;
    for(uint i = 0; i < NUM_ROBOTS; i++){
        string in = SIM_DATA + to_string(i) + ".csv";
        string out = PF_TRAJ + to_string(iteration) + "-" + to_string(i) + ".csv";
        examples.push_back(vector<Example>());

        // Run filter
        vector<vector<HA>> trajectories;
        cum_log_obs += filterFromFile(trajectories, NUM_PARTICLES, NUM_TRAJECTORIES, RESAMPLE_THRESHOLD, robots[i], in, out, dataObs[i], dataLa[i], asp);


        shuffle(begin(trajectories), end(trajectories), default_random_engine {});
        
        // Convert each particle trajectory point to EMDIPS-supported Example
        for(uint n = 0; n < SAMPLE_SIZE; n++){
            vector<HA> traj = trajectories[n];
            for(uint t = 0; t < dataObs[i].size() - 1 - END_PF_ERROR; t++){
                Example ex = dataToExample(traj[t], dataObs[i][t+1], robots[i]);

                // Provide next high-level action
                ex.result_ = SymEntry(to_string(traj[t+1]));
                if(isValidExample(ex)) examples[i].push_back(ex);
            }
        }

        // Run ASPs for all robots
        Trajectory traj (robots[i]);
        for(int j = 0; j < dataObs[i].size(); j++){
            traj.append(State { HA{}, LA{}, dataObs[i][j] });
        }
        string plot = PURE_TRAJ+to_string(iteration)+"-"+to_string(i)+".csv";
        plot_pure(traj, asp, plot);

        cout << "*";
        cout.flush();
    }

    cout << "\r";
    cout << "Cumulative observation likelihood: e^" << cum_log_obs << " = " << exp(cum_log_obs) << endl;

    return examples;
}

void default_merge(vector<vector<Example>>& allExamples, vector<Example>& consolidated){
    for(int i = 0; i < allExamples.size(); i++){
        allExamples[i] = WindowExamples(allExamples[i], WINDOW_SIZE);
    }

    for(vector<Example>& each : allExamples){
        consolidated.insert(end(consolidated), begin(each), end(each));
    }
}

// Maximization step
void maximization(vector<vector<Example>>& allExamples, uint iteration){
    vector<Example> samples;
    default_merge(allExamples, samples);

    // Set each maximum error to speed up search
    cout << "Setting error threshold to " << TARGET_LOSS << "\n\n";
    for(uint i = 0; i < transitions.size(); i++){
        accuracies[i] = TARGET_LOSS;
    }
    
    EmdipsOutput eo;
    if(iteration % STRUCT_CHANGE_FREQ == 0 && !HARDCODE_PROG){

        vector<ast_ptr> inputs; vector<Signature> sigs;
        vector<ast_ptr> ops = AST::RecEnumerateLogistic(roots, inputs, samples, library,
                                            FEATURE_DEPTH, &sigs);

        cout << "---- Number of Features Enumerated ----" << endl;
        cout << ops.size() << endl << endl;
        for(int i = 0; i < 5; i++){
            cout << ops[i] << endl;
        }
        cout << "...\n\n\n";

        // Retrieve ASPs and accuracies    
        string aspFilePath = GEN_ASP + to_string(iteration) + "/";
        filesystem::create_directory(aspFilePath);

        vector<ast_ptr> all_sketches = EnumerateL3(ops, SKETCH_DEPTH);
        
        cout << "---- Number of Total Programs ----" << endl;
        cout << all_sketches.size() << endl;
        // for(ast_ptr each: all_sketches){
        //     cout << each << endl;
        // }

        eo = emdipsL3(samples, transitions, all_sketches, preds, gt_truth, accuracies, aspFilePath, BATCH_SIZE, PROG_ENUM, false, pFunc);

    } else {
        
        // Retrieve ASPs and accuracies    
        string aspFilePath = GEN_ASP + to_string(iteration) + "/";
        filesystem::create_directory(aspFilePath);
        vector<ast_ptr> all_sketches;
        eo = emdipsL3(samples, transitions, all_sketches, preds, gt_truth, accuracies, aspFilePath, BATCH_SIZE, PROG_ENUM, true, pFunc);

    }

    
    preds = eo.ast_vec;
    accuracies = eo.log_likelihoods;

    // Write ASP info to file
    ofstream aspStrFile;
    string aspStrFilePath = GEN_ASP + to_string(iteration) + "/asp.txt";
    aspStrFile.open(aspStrFilePath);
    for(uint i = 0; i < transitions.size(); i++){
        aspStrFile << transitions[i].first + " -> " + transitions[i].second << endl;
        aspStrFile << "Accuracy: " << accuracies[i] << endl;
        aspStrFile << preds[i] << endl;
    }
    aspStrFile.close();
}

// Settings
void setupLdips(){
    cout << "-------------Setup----------------" << endl;

    Var x ("x", Dimension(1, 0, 0), NUM);
    Var v ("v", Dimension(1, -1, 0), NUM);
    Var decMax ("decMax", Dimension(1, -2, 0), NUM);
    Var vMax ("vMax", Dimension(1, -1, 0), NUM);
    Var target ("target", Dimension(1, 0, 0), NUM);

    variables.insert(x);
    variables.insert(v);
    variables.insert(decMax);
    variables.insert(vMax);
    variables.insert(target);

    if(USE_SAFE_TRANSITIONS){
        transitions.push_back(pair<string, string> ("ACC", "CON"));
        accuracies.push_back(numeric_limits<float>::max());
        transitions.push_back(pair<string, string> ("ACC", "DEC"));
        accuracies.push_back(numeric_limits<float>::max());
        transitions.push_back(pair<string, string> ("CON", "DEC"));
        accuracies.push_back(numeric_limits<float>::max());
    } else {
        for(uint i = 0; i < numHA; i++){
            for(uint j = 0; j < numHA; j++){
                transitions.push_back(pair<string, string> (to_string(to_label(i)), to_string(to_label(j))));
                accuracies.push_back(numeric_limits<float>::max());
            }
        }
    }
    
    std::sort(transitions.begin(), transitions.end(), [](const pair<string, string>& a, const pair<string, string>& b) -> bool {
        if(a.first == b.first){
            if(a.first == a.second) return false;
            if(b.first == b.second) return true;
            return a.second < b.second;
        }
        return a.first < b.first;
    });

    // Turn variables into roots
    for (const Var& variable : variables) {
        roots.push_back(make_shared<Var>(variable));
    }

    // Debug
    cout << "----Roots----" << endl;
    for (auto& node : roots) {
        cout << node << endl;
    }
    cout << endl;

    cout << "----Transitions----" << endl;
    for (auto& trans : transitions) {
        cout << trans.first << "->" << trans.second << endl;
    }
    cout << endl;

    if(HARDCODE_PROG){
        cout << "----Using fixed program----" << endl;
    } else {
        cout << "----Ground truth (target) program----" << endl;
    }
    
    // Read hard coded program structure
    for (int t = 0; t < transitions.size(); t++) {
        const auto &transition = transitions[t];
        const string input_name =
            GT_ASP_PATH + transition.first + "_" + transition.second + ".json";

        ifstream input_file;
        input_file.open(input_name);
        const json input = json::parse(input_file);
        ast_ptr fixed = AstFromJson(input);
        gt_truth.push_back(fixed);

        cout << transition.first << " -> " << transition.second << ": " << fixed << endl;
        
        input_file.close();
    }
}


void testExampleOnASP(vector<Example> examples, Robot r){
    for(uint i=0; i<examples.size(); i++){
        Example e = examples[i];
        printExampleInfo(e);
    }
}

void emLoop(vector<Robot>& robots){

    // Initialization
    setupLdips();

    library = ReadLibrary(OPERATION_LIB);
    asp* curASP = initialASP;
    vector<vector<Obs>> dataObs (NUM_ROBOTS);
    vector<vector<LA>> dataLa (NUM_ROBOTS);

    for(int r = 0; r < NUM_ROBOTS; r++){
        // Run ground truth ASP
        string inputFile = SIM_DATA + to_string(r) + ".csv";
        readData(inputFile, dataObs[r], dataLa[r]);

        // Run ASPs for all robots
        Trajectory traj (robots[r]);
        for(int j = 0; j < dataObs[r].size(); j++){
            traj.append(State { HA{}, LA{}, dataObs[r][j] });
        }
        string plot = PURE_TRAJ+"gt-"+to_string(r)+".csv";
        plot_pure(traj, ASP_model(GT_ASP), plot);
    }

    for(int i = 0; i < NUM_ITER; i++){
        
        // Expectation
        cout << "\n|-------------------------------------|\n";
        cout << "|                                     |\n";
        cout << "|          Loop " << i << " EXPECTATION         |\n";
        cout << "|                                     |\n";
        cout << "|-------------------------------------|\n";
        vector<vector<Example>> examples = expectation(i, robots, dataObs, dataLa, curASP);

        // Maximization
        cout << "\n|-------------------------------------|\n";
        cout << "|                                     |\n";
        cout << "|         Loop " << i << " MAXIMIZATION         |\n";
        cout << "|                                     |\n";
        cout << "|-------------------------------------|\n";
        maximization(examples, i);

        curASP = emdipsASP;


        // // Update point accuracy
        // double satisfied = 0;
        // double total = 0;
        // for(uint r = 0; r < NUM_ROBOTS; r++){
        //     // testExampleOnASP(examples[r], robots[r]);
        //     robots[r].POINT_ACCURACY = 1; // make ASP deterministic
        //     for(Example& ex: examples[r]){
        //         total++;
        //         Obs obs = { .pos = ex.symbol_table_["x"].GetFloat(), .vel = ex.symbol_table_["v"].GetFloat() };
        //         if(curASP(to_label(ex.start_.GetString()), obs, robots[r]) == to_label(ex.result_.GetString())){
        //             satisfied++;
        //         }
        //     }
        // }
        
        // // Update point accuracy
        // double newPointAcc = min(satisfied / total, 0.95);
        // cout << "New point accuracy: " << satisfied << " / " << total << " ~= " << newPointAcc << endl;
        // for(Robot& r : robots){
        //     r.POINT_ACCURACY = newPointAcc;
        // }
    }
}

int main() {
    // Set up Python
    // Initialize python support
    Py_Initialize();

    PyRun_SimpleString(
        "import os, sys \n"
        "sys.path.append(os.getcwd() + '/pips/src/optimizer') \n");

    // File name
    PyObject* pName = PyUnicode_FromString((char*)"optimizer");

    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        // Function name
        pFunc = PyObject_GetAttrString(pModule, (char*)"run_optimizer_threads");

        if (!(pFunc && PyCallable_Check(pFunc))) {
            if (PyErr_Occurred()) PyErr_Print();
            fprintf(stderr, "Cannot find optimization function\n");
        }
    } else {
        PyErr_Print();
        fprintf(stderr, "Failed to load optimization file");
    }

    vector<Robot> robots = getRobotSet(ROBOT_SET);
    emLoop(robots);

    // Clean up python
    Py_XDECREF(pFunc);
    Py_DECREF(pModule);
    Py_Finalize();

    return 0;
}
