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

vector<float> loss;
vector<ast_ptr> solution_preds;
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

bool use_error = true;
// Runs EMDIPS-generated ASP
HA emdipsASP(State state){
    HA prev_ha = state.ha;
    Example obsObject = dataToExample(state.ha, state.obs);
    for(uint i = 0; i < transitions.size(); i++){
        if(print(prev_ha) == transitions[i].first && transitions[i].first!=transitions[i].second){
            if(InterpretBool(solution_preds[i], obsObject)) {
                state.ha = to_label(transitions[i].second);
                break;
            }
        }
    }

    if(synthesizer == EMDIPS || !use_error) {
        return state.ha;
    }
    // Allow a little error in LDIPS
    return correct(prev_ha, pointError(prev_ha, state.ha, POINT_ACCURACY));
}

// Initial ASP: random transitions
HA initialASP(State state) {
    if(labeler == GREEDY_HEURISTIC) {
        return correct(state.ha, pointError(state.ha, state.ha, 0));
    } else {
        return correct(state.ha, pointError(state.ha, state.ha, POINT_ACCURACY));
    }
}

void save_metric(string output_path, double metric) {
    ofstream info_file;
    info_file.open(output_path, ios::app);
    info_file << metric << endl;
    info_file.close();
}

void print_metrics(double cum_log_obs, double ha_correct, double t_total, DATATYPE data) {
    cum_log_obs /= t_total; // Normalize
    ha_correct /= t_total; ha_correct *= 100; // Convert to percentage

    cout << "Metrics";
    switch(data) {
        case TRAINING:
            cout << " (training):\n"; break;
        case TESTING:
            cout << " (testing):\n"; break;
        case VALIDATION:
            cout << " (validation):\n"; break;
    }
    cout << "\tCumulative observation likelihood: e^" << cum_log_obs << " = " << exp(cum_log_obs) << endl;
    cout << "\t%% Accuracy: " << ha_correct << "%%" << endl;

    switch(data) {
        case TRAINING:
            save_metric(LOG_OBS_PATH + "-training.txt", cum_log_obs); save_metric(PCT_ACCURACY + "-training.txt", ha_correct);
            break;
        case TESTING:
            save_metric(LOG_OBS_PATH + "-testing.txt", cum_log_obs); save_metric(PCT_ACCURACY + "-testing.txt", ha_correct);
            break;
        case VALIDATION:
            save_metric(LOG_OBS_PATH + "-valid.txt", cum_log_obs); save_metric(PCT_ACCURACY + "-valid.txt", ha_correct);
            break;
    }
}

double save_pure(Trajectory traj, asp* asp, string output_path, double& ha_correct, double& ha_total) {    
    ofstream outFile;
    outFile.open(output_path + ".csv");

    map<string, shared_ptr<ofstream>> la_files;
    for(string each: LA_vars) {
        la_files[each] = make_shared<ofstream>(output_path + "-" + each + ".csv");
    }

    Trajectory gt = traj;
    double log_obs_cum = 0;

    use_error = false;

    for(uint32_t n = 0; n < SAMPLE_SIZE; n++){
        log_obs_cum += execute_pure(traj, asp);

        for(int t = 0; t < traj.size() - 1; t++){
            ha_total++;
            if(traj.get(t+1).ha == gt.get(t+1).ha){
                ha_correct++;
            }

            State last = (t == 0) ? State () : traj.get(t-1);
            State cur = traj.get(t);
            for(string each: LA_vars) {
                cur.put(each, last.get(each));
            }
            Obs la = motorModel(cur, false);
            for(string each: LA_vars) {
                (*la_files[each]) << la.get(each) << ",";
            }
        }

        Obs la = motorModel(traj.get(traj.size()-1), false);
        for(string each: LA_vars) {
            (*la_files[each]) << la.get(each) << endl;
        }

        outFile << traj.to_string();
    }
    
    outFile.close();
    for(string each: LA_vars) {
        la_files[each]->close();
    }

    use_error = true;
    return log_obs_cum;
}

// Expectation step
vector<vector<Example>> expectation(uint iteration, vector<Trajectory>& state_traj, asp* asp){

    vector<vector<Example>> examples;

    cout << "Running particle filter with " << NUM_PARTICLES << " particles\n";
    cout << "Parameters: resample threshold=" << RESAMPLE_THRESHOLD << ", observation strength=" << TEMPERATURE << endl;

    double cum_log_obs = 0;
    double ha_total = 0, ha_correct = 0;
    for(uint i = 0; i < TRAINING_SET; i++){
        string in = SIM_DATA + to_string(i) + ".csv";
        string out = TRAINING_TRAJ + to_string(iteration) + "-" + to_string(i) + ".csv";
        examples.push_back(vector<Example>());

        // Run filter
        vector<vector<HA>> trajectories;
        
        cum_log_obs += filterFromFile(trajectories, NUM_PARTICLES, SAMPLE_SIZE, in, out, state_traj[i], asp);

        shuffle(begin(trajectories), end(trajectories), default_random_engine {});
        
        // Convert each particle trajectory point to EMDIPS-supported Example
        for(uint n = 0; n < trajectories.size(); n++){
            vector<HA> traj = trajectories[n];
            for(int t = 0; t < state_traj[i].size() - 1; t++){
                Example ex = dataToExample(traj[t], state_traj[i].get(t+1).obs);

                // Provide next high-level action
                ex.result_ = SymEntry(print(traj[t+1]));
                examples[i].push_back(ex);

                ha_total++;
                if(traj[t+1] == state_traj[i].get(t+1).ha) {
                    ha_correct++;
                }
            }
        }

        cout << "*";
        cout.flush();
    }
    print_metrics(cum_log_obs, ha_correct, ha_total, TRAINING);
    
    ha_correct = ha_total = cum_log_obs = 0;
    for(uint i = 0; i < TRAINING_SET; i++){
        string plot = TESTING_TRAJ+to_string(iteration)+"-"+to_string(i);
        cum_log_obs += save_pure(state_traj[i], asp, plot, ha_correct, ha_total);
    }
    print_metrics(cum_log_obs, ha_correct, ha_total, TESTING);

    ha_correct = ha_total = cum_log_obs = 0;
    for(uint i = 0; i < VALIDATION_SET; i++){
        string plot = VALIDATION_TRAJ+to_string(iteration)+"-"+to_string(i);
        cum_log_obs += save_pure(state_traj[i], asp, plot, ha_correct, ha_total);
    }
    print_metrics(cum_log_obs, ha_correct, ha_total, VALIDATION);

    return examples;
}

void default_merge(vector<vector<Example>>& allExamples, vector<Example>& consolidated){
    for(vector<Example>& each : allExamples){
        consolidated.insert(end(consolidated), begin(each), end(each));
    }
}

// Maximization step
void maximization(vector<vector<Example>>& allExamples, uint iteration){
    vector<Example> samples;
    default_merge(allExamples, samples);
    
    // Setup output for ASPs and accuracies
    string aspFilePath = GEN_ASP + to_string(iteration) + "/";
    filesystem::create_directory(aspFilePath);

    vector<ast_ptr> inputs; vector<Signature> sigs;
    vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, samples, library,
                                        BASE_FEAT_DEPTH, &sigs);
    if (iteration % STRUCT_CHANGE_FREQ != 0) {
        // Don't enumerate: only optimize current sketch
        ops.clear();
    }

    cout << "---- Number of Features Enumerated ----" << endl;
    cout << ops.size() << endl << endl;
    for(int i = 0; i < min(5, (int) ops.size()); i++){
        cout << ops[i] << endl;
    }
    cout << "...\n\n\n";

    // Run synthesis algorithm to optimize sketches
    if(synthesizer == EMDIPS) { // EMDIPS
        emdipsL3(samples, transitions, solution_preds, loss, ops, aspFilePath, pFunc);
    } else { // LDIPS
        ldipsL3(samples, transitions, solution_preds, loss, ops, aspFilePath);
    }

    // Write ASP info to file
    ofstream aspStrFile;
    string aspStrFilePath = GEN_ASP + to_string(iteration) + "/asp.txt";
    aspStrFile.open(aspStrFilePath);
    for(uint i = 0; i < transitions.size(); i++){
        aspStrFile << transitions[i].first + " -> " + transitions[i].second << endl;
        aspStrFile << "Loss: " << loss[i] << endl;
        aspStrFile << solution_preds[i] << endl;
    }
    aspStrFile.close();
}

// Settings
void setupLdips(){
    cout << "-------------Setup----------------" << endl;

    passSettings(PROG_ENUM, PROG_COMPLEXITY_LOSS_BASE, PROG_COMPLEXITY_LOSS, synth_setting == INCREMENTAL);

    for(Var each: Obs_vars) {
        if(each.root_) {
            roots.push_back(make_shared<Var>(each));
        }
    }

    // Insert transitions
    for(uint i = 0; i < numHA; i++){
        vector<HA> valid_ha = get_valid_ha(i);
        for(uint j = 0; j < valid_ha.size(); j++){
            if(i != valid_ha[j]){
                transitions.push_back(pair<string, string> (print(i), print(valid_ha[j])));
                loss.push_back(numeric_limits<float>::max());
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
}

void read_demonstration(vector<Trajectory>& state_traj){

    cout << "\nReading demonstration and optionally running ground-truth ASP..." << endl;

    for(int r = 0; r < VALIDATION_SET; r++){
        string inputFile = SIM_DATA + to_string(r) + ".csv";
        Trajectory traj;
        readData(inputFile, traj);

        state_traj.push_back(traj);
    }

    double cum_log_obs = 0, ha_total = 0, ha_correct = 0;
    for(uint i = 0; i < TRAINING_SET; i++){
        string in = SIM_DATA + to_string(i) + ".csv";
        string out = TRAINING_TRAJ + "gt-" + to_string(i) + ".csv";

        // Run filter
        vector<vector<HA>> trajectories;
        cum_log_obs += filterFromFile(trajectories, NUM_PARTICLES, SAMPLE_SIZE, in, out, state_traj[i], ASP_model);

        for(uint n = 0; n < trajectories.size(); n++){
            for(int t = 0; t < state_traj[i].size() - 1; t++){
                ha_total++;
                if(trajectories[n][t+1] == state_traj[i].get(t+1).ha) {
                    ha_correct++;
                }
            }
        }
    }

    print_metrics(cum_log_obs, ha_correct, ha_total, TRAINING);
    
    ha_correct = ha_total = cum_log_obs = 0;
    for(uint i = 0; i < TRAINING_SET; i++){
        string plot = TESTING_TRAJ+"gt-"+to_string(i);
        cum_log_obs += save_pure(state_traj[i], ASP_model, plot, ha_correct, ha_total);
    }
    print_metrics(cum_log_obs, ha_correct, ha_total, TESTING);

    ha_correct = ha_total = cum_log_obs = 0;
    for(uint i = 0; i < VALIDATION_SET; i++){
        string plot = VALIDATION_TRAJ+"gt-"+to_string(i);
        cum_log_obs += save_pure(state_traj[i], ASP_model, plot, ha_correct, ha_total);
    }
    print_metrics(cum_log_obs, ha_correct, ha_total, VALIDATION);
}


void emLoop(){

    // Initialization
    setupLdips();

    library = ReadLibrary(OPERATION_LIB);
    asp* curASP = initialASP;
    vector<Trajectory> state_traj;
    read_demonstration(state_traj);

    for(int i = 0; i < NUM_ITER; i++){
        auto start = chrono::system_clock::now();

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
        TEMPERATURE = max(TEMPERATURE - TEMP_CHANGE, 1.0);

        auto end = chrono::system_clock::now();
        chrono::duration<double> diff = end - start;

        cout << "--------------- Iteration " << i << " took " << diff.count() << " s ---------------" << endl;
    }
}



pair<PyObject*, PyObject*> setup_python(){
    
    Py_Initialize();

    string s = "import os, sys \nsys.path.append(os.getcwd() + '/"+OPTIMIZER_PATH+"') \n";
    PyRun_SimpleString(s.c_str());

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
        fprintf(stderr, "Failed to load optimization file\n");
    }
    return pair(pFunc, pModule);
}



void cleanup_python(pair<PyObject*, PyObject*> py_info){
    auto pFunc = py_info.first;
    auto pModule = py_info.second;
    Py_XDECREF(pFunc);
    Py_DECREF(pModule);
    Py_Finalize();
}

int main() {

    auto py_info = setup_python();
    emLoop();
    cleanup_python(py_info);

    return 0;
}
