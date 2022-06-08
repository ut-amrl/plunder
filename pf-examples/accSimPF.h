#include <fstream> // for ifstream

#include <pf/resamplers.h> // for mn_resampler
#include <pf/rv_samp.h> // for sampling random numbers
#include <pf/rv_eval.h> // for evaluating densities and pmfs

#include "data_reader.h"
#include "accSim_BSmodel.h"


// some template parameters
#define dimstate 1
#define dimobs   1
#define numparts 100
#define FLOATTYPE double // choose float (faster) or double (slower)

using namespace pf::resamplers;

FLOATTYPE accMax = 20;
FLOATTYPE decMax = -10;
FLOATTYPE vMax = 5;
FLOATTYPE target = 100;

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

FLOATTYPE aspSamp(FLOATTYPE hiLvl, FLOATTYPE x, FLOATTYPE v){
    FLOATTYPE xToTarget = target - x;                                        // distance to the target
    bool cond1 = vMax - v < 0;                                           // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget - DistTraveled(v, decMax) < 0;           // needs to decelerate or else it will pass target

    bool cond1smooth = sampleDiscrete(logistic(vMax*0.1, -50.0/vMax, vMax-v));
    bool cond2smooth = sampleDiscrete(logistic(target*0.1, -50.0/target, xToTarget - DistTraveled(v, decMax)));

    double state = 0;
    if(cond2smooth){
        state = 2;
    }
    if(cond1smooth && !cond2smooth){
        state = 3;
    }
    if(!cond1smooth && !cond2smooth){
        state = 1;
    }
    return state;
}

FLOATTYPE motorModelEV(FLOATTYPE loLvl, FLOATTYPE hiLvl, FLOATTYPE obs){
    FLOATTYPE sigma = 1.0;
    FLOATTYPE acc = hiLvl == 1 ? accMax :
                    hiLvl == 2 ? decMax : 0;
    return rveval::evalUnivNorm(loLvl, acc, sigma, true);
} 

FLOATTYPE motorModelSamp(FLOATTYPE hiLvl, FLOATTYPE obs){
    rvsamp::UnivNormSampler<FLOATTYPE> normSampler;
    FLOATTYPE err = normSampler.sample();
    FLOATTYPE acc = hiLvl == 1 ? accMax :
                    hiLvl == 2 ? decMax : 0;
    return acc + err;
}

FLOATTYPE initialSamp(){
    return 3;
}

FLOATTYPE initialEV(FLOATTYPE hiLvl){
    return hiLvl==3;
}

void run_my_model(const std::string &csv)
{
    // "state size vector"
    using ssv = Eigen::Matrix<FLOATTYPE,dimstate,1>;
    // "observation sized vector"
    using osv = Eigen::Matrix<FLOATTYPE,dimobs,1>;    
    // "covariate-sized vector"
    using cvsv = Eigen::Matrix<FLOATTYPE,1,1>;
    // dynamically-sized square matrix
    using Mat = Eigen::Matrix<FLOATTYPE,Eigen::Dynamic,Eigen::Dynamic>;
    
    // model parameters that are assumed known
    FLOATTYPE phi = .91;
    FLOATTYPE beta = .5;
    FLOATTYPE sigma = 1.0;

    my_bs_wc<numparts,dimstate,dimobs,mn_resampler<numparts,dimstate,FLOATTYPE>,FLOATTYPE> bssvolwc(phi,beta,sigma,aspSamp);

    // read in some data
    std::vector<osv> data = readInData<FLOATTYPE,dimobs>(csv);

    // optional lambda
    // filter() will use this to approx.
    // an expectation now
    // in this case, the sample mean is calculated
    // at each time point, which approximates the 
    // the sequence of filtering means
    auto idtyLambda = [](const ssv& xt) -> const Mat  
    {
        return xt;
    };
    std::vector<std::function<const Mat(const ssv&)>> v;
    v.push_back(idtyLambda);

    // another identity lambda because the signature for "with covariate" models is different
    auto idtyLambdaWC = [](const ssv& xt, const cvsv& zt) -> const Mat
    {
        return xt;
    };
    std::vector<std::function<const Mat(const ssv&, const cvsv&)>> v2;
    v2.push_back(idtyLambdaWC);

    // iterate over the data (finally)
    // printing stuff is obviously optional
    for(size_t row = 0; row < data.size(); ++row){
        bssvolwc.filter(data[row], cvsv::Zero(), v2);
         
        std::cout << bssvolwc.getExpectations()[0] << ", "
                  << bssvolwc.getLogCondLike() << "\n";
    }
    
}
