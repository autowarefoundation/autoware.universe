#ifndef CPPAD_CG_TEST_FLASH_INCLUDED
#define CPPAD_CG_TEST_FLASH_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

namespace CppAD {
namespace cg {

template<class Base>
inline CppAD::ADFun<Base>* Flash(std::vector<DaeVarInfo>& daeVar,
                                 const std::vector<double>& x) {
    using namespace CppAD;
    using namespace std;
    using ADB = CppAD::AD<Base>;

    std::vector<ADB> U(15);
    assert(U.size() == x.size());
    for (size_t i = 0; i < x.size(); i++) {
        U[i] = x[i];
    }
    Independent(U);

    ADB nEthanol = U[0];
    ADB nWater = U[1];
    ADB T = U[2];
    ADB yWater = U[3];
    ADB yEthanol = U[4];
    ADB FV = U[5];
    ADB Q = U[6];
    ADB F_feed = U[7];
    ADB p = U[8];
    ADB xFEthanol = U[9];
    ADB T_feed = U[10];

    ADB D__nEthanol__Dt_1 = U[12];
    ADB D__nWater__Dt_1 = U[13];
    ADB D__T__Dt = U[14];

    daeVar.resize(U.size());
    daeVar[0] = DaeVarInfo("nEthanol");
    daeVar[1] = DaeVarInfo("nWater");
    daeVar[2] = DaeVarInfo("T");
    daeVar[3] = DaeVarInfo("yWater");
    daeVar[4] = DaeVarInfo("yEthanol");
    daeVar[5] = DaeVarInfo("FV");
    daeVar[6].makeConstant();
    daeVar[7].makeConstant();
    daeVar[8].makeConstant();
    daeVar[9].makeConstant();
    daeVar[10].makeConstant();
    daeVar[11].makeIntegratedVariable();
    daeVar[12] = 0;
    daeVar[13] = 1;
    daeVar[14] = 2;

    // dependent variable vector
    std::vector<ADB> res(6);

    ADB FNEthanol = xFEthanol * F_feed;
    ADB n_lWater = nWater * 1000.;
    ADB n_lEthanol = nEthanol * 1000.;
    ADB m_lEthanol = n_lEthanol * 0.04606844;
    ADB m_lWater = n_lWater * 0.0180152833;
    ADB m = m_lEthanol + m_lWater;
    ADB wEthanol = m_lEthanol / m;
    ADB w_Water = 1 - wEthanol;
    ADB rho = 1 / (w_Water / 983.159471259596 + wEthanol / 743.278841365274);
    ADB V = m / rho;
    ADB cWater = n_lWater / V;
    ADB dh = V / 0.502654824574367;
    ADB p_aux = p * 101325.;
    ADB dp = 101325. - p_aux;
    ADB v = sqrt((9.80665 * dh + dp / rho) * 2.);
    ADB F_VL = v * 0.0005;
    ADB FNLWater = cWater * F_VL;
    ADB n = n_lEthanol + n_lWater;
    ADB xWater = n_lWater / n;
    ADB F_NL = FNLWater / xWater;
    ADB FNLEthanol = F_NL - FNLWater;
    ADB FNVWater = yWater * FV;
    ADB FNVEthanol = FV - FNVWater;
    ADB D__nEthanol__Dt = FNEthanol - FNLEthanol - FNVEthanol;
    res[0] = D__nEthanol__Dt_1 - (D__nEthanol__Dt * 0.001);

    ADB FNFWater = F_feed - FNEthanol;
    ADB D__nWater__Dt = FNFWater - FNLWater - FNVWater;
    res[1] = D__nWater__Dt_1 - (D__nWater__Dt * 0.001);

    ADB FMFEthanol = FNEthanol * 0.04606844;
    ADB FMFWater = FNFWater * 0.0180152833;
    ADB Tfeed = T_feed + 273.15;
    ADB T_aux = T + 273.15;
    ADB dQ_F = (FMFEthanol * 2898.42878374706 + FMFWater * 4186.92536027523) * (Tfeed - T_aux);
    ADB dH_mVapEthanol = 50430. * pow(1 - T_aux / 514., 0.4989) * exp((0.4475 * T_aux) / 514.);
    ADB T_r = T_aux / 647.;
    ADB dH_mVapWater = 52053. * pow(1 - T_r, 0.3199 + -0.212 * T_r + 0.25795 * T_r * T_r);
    ADB dH_mVap = yEthanol * dH_mVapEthanol + yWater * dH_mVapWater;
    ADB dQ_vap = FV * dH_mVap;
    ADB Q_1 = Q * 1000.;
    ADB cp = wEthanol * 2898.42878374706 + w_Water * 4186.92536027523;
    res[2] = D__T__Dt - (dQ_F - dQ_vap + Q_1) / (m * cp);

    ADB pVapWater = 100000. * pow(10., 4.6543 - 1435.264 / (T_aux - 64.848));
    ADB KWater = pVapWater / p_aux;
    res[3] = yWater - xWater * KWater;

    ADB xEthanol = 1 - xWater;
    ADB pVapEthanol = exp(74.475 + -7164.3 / T_aux + -7.327 * log(T_aux) + 3.134e-06 * pow(T_aux, 2.));
    ADB KEthanol = pVapEthanol / p_aux;
    res[4] = yEthanol - xEthanol * KEthanol;
    
    res[5] = yWater + yEthanol - 1;

    // create f: U -> Z and vectors used for derivative calculations
    return new ADFun<Base> (U, res);
}

} // END cg namespace
} // END CppAD namespace

#endif