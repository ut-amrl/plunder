#ifndef my_bs_wc_H
#define my_bs_wc_H


#include <eigen3/Eigen/Dense>

#include <pf/bootstrap_filter_with_covariates.h> // the boostrap particle filter
#include <pf/rv_samp.h> // for sampling random numbers
#include <pf/rv_eval.h> // for evaluating densities and pmfs


using namespace pf;
using namespace pf::filters;
using namespace pf::bases;


template<size_t nparts, size_t dimx, size_t dimy, typename resampT, typename float_t>
class my_bs_wc : public BSFilterWC<nparts, dimx, dimy, 1, resampT, float_t>
                 , public GenForwardMod<dimx,dimy,float_t>
                 , public GenFutureSimulator<dimx,dimy,float_t,nparts>
{
public:
    using ssv = Eigen::Matrix<float_t, dimx, 1>;
    using osv = Eigen::Matrix<float_t, dimy, 1>;
    using cvsv= Eigen::Matrix<float_t,1,1>; 
 
    // parameters
    float_t m_phi;
    float_t m_beta;
    float_t m_sigma;

    // use this for sampling
    rvsamp::UnivNormSampler<float_t> m_stdNormSampler; // for sampling 
    rvsamp::UniformSampler<float_t> m_stdUniformSampler;

    // ctor
    my_bs_wc(const float_t &phi, const float_t &beta, const float_t &sigma);
    

    auto fSamp(const ssv &xtm1, const cvsv &zt) -> ssv;
    auto gSamp(const ssv &xt) -> osv;
    auto q1Samp(const osv &y1, const cvsv& z1) -> ssv;
    auto muSamp() -> ssv;

    float_t logGEv(const osv &yt, const ssv &xt, const cvsv& zt);
    float_t logQ1Ev(const ssv &x1, const osv &y1, const cvsv &z1);
    float_t logMuEv(const ssv &x1, const cvsv &z1);


    std::array<ssv,nparts> get_uwtd_samps() const;
};



// MAIN CONSTRUCTOR ----------------------------
// PASS IN ASP (DISTRIBUTION: (HIGH-LEVEL, OBS-STATE) -> (NEXT-HI-LEVEL))
// PASS IN MOTOR-MODEL (DISTRIBUTION: (HI-LEVEL, OBS-STATE) -> (LO-LEVEL))
// PASS IN INITIAL-HI-LEVEL DISTRIBUTION
template<size_t nparts, size_t dimx, size_t dimy, typename resampT, typename float_t>
my_bs_wc<nparts, dimx, dimy, resampT, float_t>::my_bs_wc(const float_t &phi, const float_t &beta, const float_t &sigma) 
    : m_phi(phi), m_beta(beta), m_sigma(sigma)
{
}



// APPLY INITIAL DISTRIBUTION (SAMPLE FROM TIME=1) ---------------------------
// OSV Y1 = OBSERVATION SIZE VECTOR = LO-LEVEL AT T=1 (?)
// CVSV Z1 = COVARIATE SIZE VECTOR = OBS-STATE AT T=1 (?)
// RETURN SSV = STATE SIZE VECTOR = HI-LEVEL
template<size_t nparts, size_t dimx, size_t dimy, typename resampT, typename float_t>
auto my_bs_wc<nparts, dimx, dimy, resampT, float_t>::q1Samp(const osv &y1, const cvsv &z1) -> ssv
{
    ssv x1samp;
    x1samp(0) = m_stdUniformSampler.sample();
    return x1samp;
}



// APPLY ASP (SAMPLE FROM HI-LEVEL TRANSITION DISTRIBUTION) ---------------------
// SSV XTM1 = HI-LEVEL AT TIME=T-1
// CVSV ZT = OBS-STATE AT TIME=T
// RETURN SSV = HI-LEVEL AT TIME=T
template<size_t nparts, size_t dimx, size_t dimy, typename resampT, typename float_t>
auto my_bs_wc<nparts, dimx, dimy, resampT, float_t>::fSamp(const ssv &xtm1, const cvsv &zt) -> ssv
{
    ssv xtsamp;
    xtsamp(0) = xtm1(0) + m_stdNormSampler.sample();
    return xtsamp;
}


// APPLY MOTOR-MODEL (PROBABILITY OF LO-LEVEL GIVEN HI-LEVEL) -----------------
// OSV YT = LO-LEVEL
// SSV XT = HI-LEVEL
// CVSV ZT = OBS-STATE (?)
// RETURN PROBABILITY
template<size_t nparts, size_t dimx, size_t dimy, typename resampT, typename float_t>
float_t my_bs_wc<nparts, dimx, dimy, resampT, float_t>::logGEv(const osv &yt, const ssv &xt, const cvsv& zt)
{
    return rveval::evalUnivNorm<float_t>(
				   yt(0),
                                   0.0,
                                   m_beta * std::exp(.5*xt(0)),
                                   true);
}



// APPLY MOTOR-MODEL (LO-LEVEL GIVEN HI-LEVEL) -----------------
// SSV XT = HI-LEVEL
// RETURN OSV YT = LO-LEVEL
template<size_t nparts, size_t dimx, size_t dimy, typename resampT, typename float_t>
auto my_bs_wc<nparts, dimx, dimy, resampT, float_t>::gSamp(const ssv &xt) -> osv {
    osv yt;
    yt(0) = m_stdNormSampler.sample() * m_beta * std::exp(.5*xt(0));
    return yt;
}



// HOW LIKELY IS X1 GIVEN Z1 ----------------------------
template<size_t nparts, size_t dimx, size_t dimy, typename resampT, typename float_t>
float_t my_bs_wc<nparts, dimx, dimy, resampT, float_t>::logMuEv(const ssv &x1, const cvsv& z1)
{
    return rveval::evalUnivNorm<float_t>(x1(0),
                                   0.0,
                                   m_sigma/std::sqrt(1.0 - m_phi*m_phi),
                                   true);
}



// SAME AS Q1SAMP FOR SOME REASON? ---------------------------
template<size_t nparts, size_t dimx, size_t dimy, typename resampT, typename float_t>
auto my_bs_wc<nparts, dimx, dimy, resampT, float_t>::muSamp() -> ssv {

    ssv x1; 
    x1(0) = m_stdNormSampler.sample() * m_sigma/std::sqrt(1.0 - m_phi*m_phi);
    return x1;
}


// HOW LIKELY IS X1 GIVEN Y1 AND Z1
template<size_t nparts, size_t dimx, size_t dimy, typename resampT, typename float_t>
float_t my_bs_wc<nparts, dimx, dimy, resampT, float_t>::logQ1Ev(const ssv &x1samp, const osv &y1, const cvsv& z1)
{
    return rveval::evalUnivNorm<float_t>(x1samp(0), 0.0, m_sigma/std::sqrt(1.0 - m_phi*m_phi), true);
}


template<size_t nparts, size_t dimx, size_t dimy, typename resampT, typename float_t>    
auto my_bs_wc<nparts, dimx, dimy, resampT, float_t>::get_uwtd_samps() const -> std::array<ssv,nparts> 
{
    return this->m_particles;
}

#endif //my_bs_wc_H
