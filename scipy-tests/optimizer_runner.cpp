#include <Python.h>

#include <iomanip>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>

using namespace std;

int main(int argc, char *argv[])
{   
    PyObject *pName, *pModule, *pFunc, *pArgs, *pValue;

    Py_Initialize();
    PyRun_SimpleString("import sys; sys.path.insert(0, './')");

    // File name
    pName = PyUnicode_FromString((char*)"py_function");

    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if(pModule != NULL) {

        // Function name
        pFunc = PyObject_GetAttrString(pModule, (char*)"test");

        if(pFunc && PyCallable_Check(pFunc)) {
            // Most of the function calling logic

            // Arguments
            pArgs = PyTuple_Pack(1, PyUnicode_FromString((char*)"Greg"));

            if (!pValue) {
                fprintf(stderr, "Cannot convert argument\n");
                return 1;
            }

            // Call function
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);

            if (pValue != NULL) {
                // Retrieve results
                auto result = _PyUnicode_AsString(pValue);
                cout << result << endl;
            } else {
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return 1;
            }

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

    return 0;
}