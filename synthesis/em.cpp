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
            if(InterpretBool(solution_preds[i], obsObject)) {
                state.ha = to_label(transitions[i].second);
                break;
            }
        }
    }

    return state.ha;
    // return correct(prev_ha, pointError(prev_ha, state.ha, POINT_ACCURACY)); // Introduce point errors - random transitions allow GT_ASP to escape local minima
}

// Initial ASP: random transitions
HA initialASP(State state) {
    return correct(state.ha, pointError(state.ha, state.ha, POINT_ACCURACY));
}

double save_pure(Trajectory traj, asp* asp, string output_path) {
    ofstream outFile;
    outFile.open(output_path);

    double log_obs_cum = 0;

    for(uint32_t n = 0; n < PARTICLES_PLOTTED; n++){
        log_obs_cum += execute_pure(traj, asp);
        outFile << traj.to_string();
    }

    outFile.close();
    return log_obs_cum;
}

// Expectation step
vector<vector<Example>> expectation(uint iteration, vector<Trajectory>& state_traj, asp* asp){

    vector<vector<Example>> examples;

    cout << "Running particle filter with " << NUM_PARTICLES << " particles\n";
    cout << "Parameters: resample threshold=" << RESAMPLE_THRESHOLD << ", observation strength=" << TEMPERATURE << endl;

    double cum_log_obs_pf = 0;
    for(uint i = 0; i < TRAINING_SET; i++){
        string in = SIM_DATA + to_string(i) + ".csv";
        string out = PF_TRAJ + to_string(iteration) + "-" + to_string(i) + ".csv";
        examples.push_back(vector<Example>());

        // Run filter
        vector<vector<HA>> trajectories;
        
        cum_log_obs_pf += filterFromFile(trajectories, NUM_PARTICLES, max(SAMPLE_SIZE, PARTICLES_PLOTTED), in, out, state_traj[i], asp);


        shuffle(begin(trajectories), end(trajectories), default_random_engine {});
        
        // Convert each particle trajectory point to EMDIPS-supported Example
        for(uint n = 0; n < SAMPLE_SIZE; n++){
            vector<HA> traj = trajectories[n];
            for(int t = 0; t < state_traj[i].size() - 1 - END_PF_ERROR; t++){
                Example ex = dataToExample(traj[t], state_traj[i].get(t+1).obs);

                // Provide next high-level action
                ex.result_ = SymEntry(print(traj[t+1]));
                examples[i].push_back(ex);
            }
        }

        cout << "*";
        cout.flush();
    }

    double cum_log_obs_pure = 0;
    for(uint i = 0; i < VALIDATION_SET; i++){
        string plot = PURE_TRAJ+to_string(iteration)+"-"+to_string(i)+".csv";
        cum_log_obs_pure += save_pure(state_traj[i], asp, plot);
    }

    cout << "\r";
    cout << "Cumulative observation likelihood (particle filter): e^" << cum_log_obs_pf << " = " << exp(cum_log_obs_pf) << endl;
    cout << "Cumulative observation likelihood (pure outputs): e^" << cum_log_obs_pure << " = " << exp(cum_log_obs_pure) << endl;
    
    ofstream info_file_pf;
    info_file_pf.open(INFO_FILE_PATH + "-pf.txt", ios::app);
    info_file_pf << cum_log_obs_pf << endl;
    info_file_pf.close();

    ofstream info_file_pure;
    info_file_pure.open(INFO_FILE_PATH + "-pure.txt", ios::app);
    info_file_pure << cum_log_obs_pure << endl;
    info_file_pure.close();

    return examples;
}

void default_merge(vector<vector<Example>>& allExamples, vector<Example>& consolidated){
    for(uint i = 0; i < allExamples.size(); i++){
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
        loss[i] = TARGET_LOSS;
    }
    
    // Setup output for ASPs and accuracies
    string aspFilePath = GEN_ASP + to_string(iteration) + "/";
    filesystem::create_directory(aspFilePath);

    if(iteration % STRUCT_CHANGE_FREQ == 0){

        vector<ast_ptr> inputs; vector<Signature> sigs;
        vector<ast_ptr> ops = AST::RecEnumerateLogistic(roots, inputs, samples, library,
                                            FEATURE_DEPTH, &sigs);

        cout << "---- Number of Features Enumerated ----" << endl;
        cout << ops.size() << endl << endl;
        for(int i = 0; i < min(5, (int) ops.size()); i++){
            cout << ops[i] << endl;
        }
        cout << "...\n\n\n";

        vector<ast_ptr> all_sketches = EnumerateL3(ops, SKETCH_DEPTH);
        
        cout << "---- Number of Total Programs ----" << endl;
        cout << all_sketches.size() << endl;
        if(DEBUG) {
            for(int i = 0; i < min(10, (int) all_sketches.size()); i++){
                cout << all_sketches[i] << endl;
            }
            cout << "...\n\n";
        }

        emdipsL3(samples, transitions, solution_preds, loss, all_sketches, solution_preds, gt_truth, loss, aspFilePath, BATCH_SIZE, PROG_ENUM, true, PROG_COMPLEXITY_LOSS, pFunc);

    } else {
        
        // Retrieve ASPs and accuracies
        vector<ast_ptr> all_sketches;
        emdipsL3(samples, transitions, solution_preds, loss, all_sketches, solution_preds, gt_truth, loss, aspFilePath, BATCH_SIZE, PROG_ENUM, false, PROG_COMPLEXITY_LOSS, pFunc);

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

    for(Var each: Obs_vars) {
        if(each.root_) {
            roots.push_back(make_shared<Var>(each));
        }
    }

    // Insert transitions
    for(uint i = 0; i < numHA; i++){
        vector<HA> valid_ha = get_valid_ha(i);
        for(uint j = 0; j < valid_ha.size(); j++){
            if(i != j){
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

    if(GT_PRESENT) {
        cout << "----Ground truth (target) program----" << endl;
     
        // Read hard coded program structure
        for (uint t = 0; t < transitions.size(); t++) {
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
}

void testExampleOnASP(vector<Example> examples){
    for(uint i=0; i<examples.size(); i++){
        printExampleInfo(examples[i]);
    }
}

void read_demonstration(vector<Trajectory>& state_traj){

    cout << "\nReading demonstration and optionally running ground-truth ASP..." << endl;

    double cum_log_obs_pf = 0;
    double cum_log_obs_pure = 0;
    for(int r = 0; r < VALIDATION_SET; r++){
        string inputFile = SIM_DATA + to_string(r) + ".csv";
        Trajectory traj;
        readData(inputFile, traj);

        // Run and plot ground truth ASP
        if(GT_PRESENT){
            string plot = PURE_TRAJ+"gt-"+to_string(r)+".csv";
            cum_log_obs_pure += save_pure(traj, ASP_model, plot);

            if(r < TRAINING_SET) {
                string in = SIM_DATA + to_string(r) + ".csv";
                string out = PF_TRAJ + "gt" + "-" + to_string(r) + ".csv";

                // Run filter
                vector<vector<HA>> trajectories;
                cum_log_obs_pf += filterFromFile(trajectories, NUM_PARTICLES, max(SAMPLE_SIZE, PARTICLES_PLOTTED), in, out, traj, ASP_model);
            }
        }

        state_traj.push_back(traj);
    }

    cout << "Target cumulative observation likelihood (ground truth, particle filter): e^" << cum_log_obs_pf << " = " << exp(cum_log_obs_pf) << endl;
    cout << "Target cumulative observation likelihood (ground truth, pure): e^" << cum_log_obs_pure << " = " << exp(cum_log_obs_pure) << endl;

    ofstream info_file_pf;
    info_file_pf.open(INFO_FILE_PATH + "-pf.txt", ios::app);
    info_file_pf << cum_log_obs_pf << endl;
    info_file_pf.close();

    ofstream info_file_pure;
    info_file_pure.open(INFO_FILE_PATH + "-pure.txt", ios::app);
    info_file_pure << cum_log_obs_pure << endl;
    info_file_pure.close();
}


void emLoop(){

    // Initialization
    setupLdips();

    library = ReadLibrary(OPERATION_LIB);
    asp* curASP = initialASP;
    vector<Trajectory> state_traj;
    read_demonstration(state_traj);

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
        TEMPERATURE = max(TEMPERATURE + TEMP_CHANGE, 1.0);
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
