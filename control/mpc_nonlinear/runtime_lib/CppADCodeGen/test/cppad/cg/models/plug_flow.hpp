#ifndef CPPAD_CG_TEST_PLUG_FLOW_INCLUDED
#define CPPAD_CG_TEST_PLUG_FLOW_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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

#include <assert.h>

namespace CppAD {

template<class Base>
class PlugFlowModel {
public:
    static const size_t N_EL_STATES = 5; // number of states per element
    static const size_t N_CONTROLS = 2; // number of controls
    static const size_t N_PAR = 5; // number of parameters
public:
    const Base logAk0;
    const Base Ea; // activation energy
    const Base R;
    const Base CpA; // heat capacity of A
    const Base CpB; // heat capacity of B
    const Base CpC; // heat capacity of C
    const Base CpW; // heat capacity of water
    const Base Cpcool; // heat capacity of the coolant
    const Base rho; // water specific mass
    const Base rhoCool; // coolant specific mass
    const Base Mw; // molar mass of water
    const Base Ma; // molar mass of A
    const Base Mb; // molar mass of B
    const Base Mc; // molar mass of C
    const Base Mcool; // molar mass of Coolant
    const Base r0; // inner tube inner radius;
    const Base r1; // inner tube outer radius;
    const Base r2; // outer tube inner radius;
    const Base L; // cell length
    const Base U;
public:

    PlugFlowModel() :
        logAk0(std::log(2.0e12)),
        Ea(10300.0),
        R(8.31447215),
        CpA(1000.),
        CpB(1000.),
        CpC(2000.),
        CpW(4189.68014953824),
        Cpcool(4189.68014953824),
        rho(1001.91585067007),
        rhoCool(1001.91585067007),
        Mw(0.0180152833),
        Ma(0.05),
        Mb(0.07),
        Mc(0.12),
        Mcool(0.12),
        r0(0.03),
        r1(0.033),
        r2(0.045),
        L(5. / 20.),
        U(1. / (1. / 1000. + (r1 - r0) / 20. + 1 / 1000.)) {
    }

    static std::vector<std::set<size_t> > getRelatedCandidates(size_t nEls = 6) {
        std::vector<std::set<size_t> > relatedDepCandidates(N_EL_STATES);
        for (size_t i = 0; i < nEls; i++) {
            relatedDepCandidates[0].insert(0 * nEls + i);
            relatedDepCandidates[1].insert(1 * nEls + i);
            relatedDepCandidates[2].insert(2 * nEls + i);
            relatedDepCandidates[3].insert(3 * nEls + i);
            relatedDepCandidates[4].insert(4 * nEls + i);
        }
        return relatedDepCandidates;
    }

    static std::vector<double> getTypicalValues(size_t nEls = 6) {
        size_t ns = N_EL_STATES;
        size_t nm = N_CONTROLS;

        std::vector<double> x(ns * nEls + nm + N_PAR);

        // states
        for (size_t j = 0; j < nEls; j++) x[j] = std::sqrt(14 / 1000.) - j * 0.01; // Ca (mol/l)
        for (size_t j = 0; j < nEls; j++) x[1 * nEls + j] = 1.2 - j * 0.01; // Cb (mol/l)
        for (size_t j = 0; j < nEls; j++) x[2 * nEls + j] = j * 0.01; // Cc (mol/l)
        for (size_t j = 0; j < nEls; j++) x[3 * nEls + j] = 50 + j * 0.1; // T (C)
        for (size_t j = 0; j < nEls; j++) x[4 * nEls + j] = 10 + (nEls - j) * 0.1; // T (C)

        size_t nst = nEls * ns;

        //controls
        x[nst] = 7; // Fin (l/min)
        x[nst + 1] = 7; // FCoolin (l/min)

        //parameters
        size_t p = nst + nm;
        x[p + 0] = std::sqrt(14 / 1000.); // Ca0 (mol/l)
        x[p + 1] = 0.1; // Cb0 (mol/l)
        x[p + 2] = 0.0; // Cc0 (mol/l)
        x[p + 3] = 20; // T0 (C)
        x[p + 4] = 10; // TCool0 (C)

        return x;
    }

    std::vector<CppAD::AD<Base> > model(const std::vector<CppAD::AD<Base> >& x,
                                        size_t nEls = 6) {
        using namespace CppAD;
        using ADB = AD<Base>;

        // dependent variable vector
        std::vector<ADB> y(N_EL_STATES * nEls);

        size_t ns = N_EL_STATES * nEls;
        size_t nm = N_CONTROLS;
        size_t pars = ns + nm; // initial index of parameters in independent variables

        ADB F = (1e-3 / 60.) * x[ns]; // convert from l/min
        ADB Fcool = (1e-3 / 60.) * x[ns + 1]; // convert from l/min
        ADB Vin = L * 3.14159265358979 * r0 * r0;
        ADB Vout = L * 3.14159265358979 * r2 * r2 - L * 3.14159265358979 * r1 * r1;
        ADB area = L * 6.28318530717959 * r0;

        for (size_t j = 0; j < nEls; j++) {

            ADB Ca0 = 1000. * x[(j == 0) ? pars : j + -1]; // covert from mol/l
            ADB Cb0 = 1000. * x[(j == 0) ? pars + 1 : j + nEls - 1]; // covert from mol/l
            ADB Cc0 = 1000. * x[(j == 0) ? pars + 2 : j + 2 * nEls - 1]; // covert from mol/l

            ADB Ca1 = 1000. * x[j]; // covert from mol/l
            ADB Cb1 = 1000. * x[j + nEls]; // covert from mol/l
            ADB Cc1 = 1000. * x[j + 2 * nEls]; // covert from mol/l

            ADB Fina = Ca0 * F;
            ADB Finb = Cb0 * F;
            ADB Finc = Cc0 * F;

            ADB Tin = x[(j == 0) ? pars + 3 : j + 3 * nEls - 1] - -273.15; // convert from C
            ADB T = x[j + 3 * nEls] - -273.15; // convert from C
            ADB Tcool = x[j + 4 * nEls] - -273.15; // convert from C
            ADB TcoolIn = x[(j == nEls - 1) ? pars + 4 : j + 4 * nEls + 1] - -273.15; // convert from C

            ADB react = exp(logAk0 - Ea / (R * T)) * Ca1 * Cb1;

            y[j] = 0.001 * (Fina - Ca1 * F - react * Vin) / Vin; // d CA / dt
            y[j + nEls] = 0.001 * (Finb - Cb1 * F - react * Vin) / Vin; // d CB / dt
            y[j + 2 * nEls] = 0.001 * (Finc - Cc1 * F + react * Vin) / Vin; // d CC / dt

            ADB Cw0 = (rho - (Ma * Ca0 + Mb * Cb0 + Mc * Cc0)) / Mw;
            ADB Cw1 = (rho - (Ma * Ca1 + Mb * Cb1 + Mc * Cc1)) / Mw;
            ADB CpMix = (CpA * Ma * Ca1 + CpB * Mb * Cb1 + CpC * Mc * Cc1 + CpW * Mw * Cw1) / (Ma * Ca1 + Mb * Cb1 + Mc * Cc1 + Mw * Cw1);

            ADB dQ = U * area * (T - Tcool);

            ADB CpFin = (Ma * Fina * CpA + Mb * Finb * CpB + Mc * Finc * CpC + Mw * Cw0 * F * CpW);

            y[j + 3 * nEls] = (CpFin * (Tin - T) - -33488. * react * Vin - dQ) /
                    (rho * Vin * CpMix); // dT / dt

            y[j + 4 * nEls] = (Cpcool * rhoCool * Fcool * (TcoolIn - Tcool) + dQ) /
                    (rhoCool * Vout * Cpcool); // dTcool / dt
        }

        return y;
    }

    /**
     * Constant mixture heat capacity and rho
     */
    std::vector<CppAD::AD<Base> > model2(const std::vector<CppAD::AD<Base> >& x,
                                         size_t nEls = 6) {
        using namespace CppAD;
        using ADB = AD<Base>;

        // dependent variable vector
        std::vector<ADB> y(N_EL_STATES * nEls);

        size_t ns = N_EL_STATES * nEls;
        size_t nm = N_CONTROLS;
        size_t pars = ns + nm; // initial index of parameters in independent variables

        double dHr = -33488.;

        ADB F = (1e-3 / 60.) * x[ns]; // convert from l/min
        ADB Fcool = (1e-3 / 60.) * x[ns + 1]; // convert from l/min
        ADB Vin = L * 3.14159265358979 * r0 * r0;
        ADB Vout = L * 3.14159265358979 * r2 * r2 - L * 3.14159265358979 * r1 * r1;
        ADB area = L * 6.28318530717959 * r0;

        ADB FMcool = Fcool * rhoCool;
        ADB FMfeed = F * rho;

        for (size_t j = 0; j < nEls; j++) {

            ADB Ca0 = 1000. * x[(j == 0) ? pars : j + -1]; // covert from mol/l
            ADB Cb0 = 1000. * x[(j == 0) ? pars + 1 : j + nEls - 1]; // covert from mol/l
            ADB Cc0 = 1000. * x[(j == 0) ? pars + 2 : j + 2 * nEls - 1]; // covert from mol/l

            ADB Ca1 = 1000. * x[j]; // covert from mol/l
            ADB Cb1 = 1000. * x[j + nEls]; // covert from mol/l
            ADB Cc1 = 1000. * x[j + 2 * nEls]; // covert from mol/l

            ADB FinA = Ca0 * F;
            ADB FinB = Cb0 * F;
            ADB FinC = Cc0 * F;

            ADB FA = Ca1 * F;
            ADB FB = Cb1 * F;
            ADB FC = Cc1 * F;

            ADB Tin = x[(j == 0) ? pars + 3 : j + 3 * nEls - 1] - -273.15; // convert from C
            ADB T = x[j + 3 * nEls] - -273.15; // convert from C
            ADB Tcool = x[j + 4 * nEls] - -273.15; // convert from C
            ADB TcoolIn = x[(j == nEls - 1) ? pars + 4 : j + 4 * nEls + 1] - -273.15; // convert from C
            ADB react = exp(logAk0 - Ea / (R * T)) * Ca1 * Cb1;

            y[j] = 0.001 * (FinA - FA - react * Vin) / Vin; // d CA / dt
            y[j + nEls] = 0.001 * (FinB - FB - react * Vin) / Vin; // d CB / dt
            y[j + 2 * nEls] = 0.001 * (FinC - FC + react * Vin) / Vin; // d CC / dt

            ADB FMin = FMfeed;
            ADB Qinner0 = FMin * CpW * (Tin - T);

            ADB CpMix1 = CpW;

            ADB dQ = U * area * (T - Tcool);

            ADB mInner = rho * Vin;
            y[j + 3 * nEls] = (Qinner0 - dQ - dHr * react * Vin) /
                              (mInner * CpMix1); // dT / dt

            ADB Qouter = FMcool * Cpcool * (TcoolIn - Tcool);
            ADB mCool = rhoCool * Vout;
            y[j + 4 * nEls] = (Qouter + dQ) /
                              (mCool * Cpcool); // dTcool / dt
        }

        return y;
    }

};

}

#endif
