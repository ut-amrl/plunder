#include "particleFilter/pf_runner.h"
#include "simulation/generate.h"

using namespace std;
using namespace AST;
using namespace SETTINGS;
using Eigen::Vector2f;
using json = nlohmann::json;

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
Example dataToExample(HA ha, Obs state){
    Example ex;

    for(Var each: Obs_vars) {
        if(each.root_) {
            ex.symbol_table_[each.name_] = SymEntry((float) state.get(each.name_));
        }
    }

    ex.start_ = SymEntry(print(ha));

    return ex;
}

void printExampleInfo(Example e){
    string start = e.start_.GetString();
    string res = e.result_.GetString();
    cout << start << "->" << res;
    for(Var each: Obs_vars) {
        if(each.root_){
            cout << ", " << each.name_ << " : " << e.symbol_table_[each.name_].GetFloat();
        }
    }
    cout << endl;
}

// Runs EMDIPS-generated ASP
HA emdipsASP(State state){
    HA prev_ha = state.ha;
    Example obsObject = dataToExample(state.ha, state.obs);
    for(uint i = 0; i < transitions.size(); i++){
        if(print(prev_ha) == transitions[i].first && transitions[i].first!=transitions[i].second){
            if(InterpretBool(preds[i], obsObject)) {
                state.ha = to_label(transitions[i].second);
                break;
            }
        }
    }

    return pointError(prev_ha, state.ha, POINT_ACCURACY, USE_SAFE_TRANSITIONS); // Introduce point errors - random transitions allow GT_ASP to escape local minima
}

// Initial ASP: random transitions
HA initialASP(State state) {
    return pointError(state.ha, state.ha, POINT_ACCURACY, USE_SAFE_TRANSITIONS);
}

void plot_pure(Trajectory& traj, asp* asp, string output_path) {

    ofstream outFile;
    outFile.open(output_path);

    for(uint32_t n = 0; n < PARTICLES_PLOTTED; n++){
        execute_pure(traj, asp);

        for(uint32_t t = 0; t < traj.size() - 1; t++) {
            outFile << traj.get(t).ha << ",";
        }
        outFile << traj.get(traj.size() - 1).ha << endl;

    }
    outFile.close();
}

// Expectation step
vector<vector<Example>> expectation(uint iteration, vector<Trajectory>& state_traj, asp* asp){

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
        cum_log_obs += filterFromFile(trajectories, NUM_PARTICLES, NUM_TRAJECTORIES, in, out, state_traj[i], asp);


        shuffle(begin(trajectories), end(trajectories), default_random_engine {});
        
        // Convert each particle trajectory point to EMDIPS-supported Example
        for(uint n = 0; n < SAMPLE_SIZE; n++){
            vector<HA> traj = trajectories[n];
            for(uint t = 0; t < state_traj[i].size() - 1 - END_PF_ERROR; t++){
                Example ex = dataToExample(traj[t], state_traj[i].get(t+1).obs);

                // Provide next high-level action
                ex.result_ = SymEntry(print(traj[t+1]));
                examples[i].push_back(ex);
            }
        }

        string plot = PURE_TRAJ+to_string(iteration)+"-"+to_string(i)+".csv";
        plot_pure(state_traj[i], asp, plot);

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

    for(Var each: Obs_vars) {
        if(each.root_) {
            roots.push_back(make_shared<Var>(each));
        }
    }

    // Insert transitions
    for(uint i = 0; i < numHA; i++){
        vector<HA> valid_ha = get_valid_ha(i, USE_SAFE_TRANSITIONS);
        for(uint j = 0; j < valid_ha.size(); j++){
            transitions.push_back(pair<string, string> (print(i), print(valid_ha[j])));
            accuracies.push_back(numeric_limits<float>::max());
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

// TODO: this is deprecated
void update_point_accuracies(vector<Robot>& robots, vector<vector<Example>>& examples){
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


void testExampleOnASP(vector<Example> examples, Robot r){
    for(uint i=0; i<examples.size(); i++){
        Example e = examples[i];
        printExampleInfo(e);
    }
}

void emLoop(){

    // Initialization
    setupLdips();

    library = ReadLibrary(OPERATION_LIB);
    asp* curASP = initialASP;
    vector<Trajectory> state_traj;

    for(int r = 0; r < NUM_ROBOTS; r++){
        // Run ground truth ASP
        string inputFile = SIM_DATA + to_string(r) + ".csv";
        Trajectory traj;
        readData(inputFile, traj);

        // Run ASPs for all robots
        string plot = PURE_TRAJ+"gt-"+to_string(r)+".csv";
        plot_pure(traj, ASP_model(GT_ASP), plot);

        state_traj.push_back(traj);
    }

    for(int i = 0; i < NUM_ITER; i++){
        
        // Expectation
        cout << "\n|-------------------------------------|\n";
        cout << "|                                     |\n";
        cout << "|          Loop " << i << " EXPECTATION         |\n";
        cout << "|                                     |\n";
        cout << "|-------------------------------------|\n";
        vector<vector<Example>> examples = expectation(i, state_traj, curASP);

        // Maximization
        cout << "\n|-------------------------------------|\n";
        cout << "|                                     |\n";
        cout << "|         Loop " << i << " MAXIMIZATION         |\n";
        cout << "|                                     |\n";
        cout << "|-------------------------------------|\n";
        maximization(examples, i);

        curASP = emdipsASP;

        // update_point_accuracies(robots, examples);
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

    emLoop();

    // Clean up python
    Py_XDECREF(pFunc);
    Py_DECREF(pModule);
    Py_Finalize();

    return 0;
}
