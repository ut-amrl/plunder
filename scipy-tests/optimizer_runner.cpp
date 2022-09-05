#include <Python.h>

#include <iomanip>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <vector>

using namespace std;

int main(int argc, char *argv[])
{   
    // Test Data
    vector<char> clauses = { '&' };
    vector<bool> y_j = { false, false, false, true, true, true, true, false, false, false, false };
    vector<vector<float>> E_k = {
        {-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49},
        {-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49}
    };

    vector<float> a_vals;
    vector<float> x_0_vals;
    float sol = 0.0;

    // Initialization
    Py_Initialize();
    PyRun_SimpleString("import sys; sys.path.insert(0, './')");

    PyObject *pName, *pModule, *pFunc, *pArgs, *pValue;
    PyObject *pE_k, *pY_j, *pClauses;

    // Arguments
    pClauses = PyList_New(clauses.size());
    for(int i = 0; i < clauses.size(); i++){
        PyList_SetItem(pClauses, i, PyUnicode_FromString(&clauses[i]));
    }
    pY_j = PyList_New(y_j.size());
    for(int i = 0; i < y_j.size(); i++){
        PyList_SetItem(pY_j, i, (y_j[i] ? Py_True : Py_False));
    }

    pE_k = PyList_New(E_k.size());
    for(int i = 0; i < E_k.size(); i++){
        PyObject *subarr;
        subarr = PyList_New(E_k[i].size());
        for(int j = 0; j < E_k[i].size(); j++){
            PyList_SetItem(subarr, j, PyFloat_FromDouble(E_k[i][j]));
        }
        PyList_SetItem(pE_k, i,  subarr);
    }
    pArgs = PyTuple_Pack(3, pE_k, pY_j, pClauses);

    if (!pArgs) {
        fprintf(stderr, "Cannot convert argument\n");
        return 1;
    }

    // File name
    pName = PyUnicode_FromString((char*)"optimizer");

    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if(pModule != NULL) {

        // Function name
        pFunc = PyObject_GetAttrString(pModule, (char*)"run_optimizer");

        if(pFunc && PyCallable_Check(pFunc)) {
            // Call function
            pValue = PyObject_CallObject(pFunc, pArgs);

            if (pValue != NULL && PyTuple_Check(pValue)) {
                // Retrieve results
                sol = PyFloat_AsDouble(PyTuple_GetItem(pValue, 0));
                pValue = PyTuple_GetItem(pValue, 1);
                for(int i = 0; i < PyList_Size(pValue) / 2; i++){
                    a_vals.push_back(PyFloat_AsDouble(PyList_GetItem(pValue, i)));
                }
                for(int i = PyList_Size(pValue) / 2; i < PyList_Size(pValue); i++){
                    x_0_vals.push_back(PyFloat_AsDouble(PyList_GetItem(pValue, i)));
                }
            } else {
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return 1;
            }

            Py_DECREF(pArgs);
            Py_DECREF(pValue);
        } else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", argv[2]);
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);

    } else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", argv[1]);
        return 1;
    }
    
    Py_Finalize();


    cout << "Total error: " << sol << endl;
    for(int i = 0; i < a_vals.size(); i++){
        cout << "logistic(" << a_vals[i] << ", " << x_0_vals[i] << "); ";
    }
    cout << endl;

    return 0;
}