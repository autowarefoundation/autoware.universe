#ifndef CPPAD_CG_TEST_TANK_BATTERY_INCLUDED
#define CPPAD_CG_TEST_TANK_BATTERY_INCLUDED
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

template<class T>
std::vector<CppAD::AD<T> > tankBatteryFunc(const std::vector<CppAD::AD<T> >& x) {
    using namespace CppAD;

    // dependent variable vector 
    std::vector< AD<T> > dhdt(6);

    // temporary variables
    std::vector< AD<T> > v(3);

    T g = 9.80665;
    T pi = 3.1415926535897931;
    AD<T> r = x[7];

    v[0] = pi * r * r;
    v[1] = (1001.9158506700721 * 0.0005 * sqrt(2. * g * (x[0] / 1001.9158506700721) / v[0])) / 0.0180152833;
    dhdt[0] = 0.0180152833 * ((1001.9158506700721 * 0.016666666666666666 * x[6]) / 0.0180152833 - v[1]);

    v[2] = (1001.9158506700721 * 0.0005 * sqrt(2. * g * (x[1] / 1001.9158506700721) / v[0])) / 0.0180152833;
    dhdt[1] = 0.0180152833 * (v[1] - v[2]);

    v[1] = (1001.9158506700721 * 0.0005 * sqrt(2. * g * (x[2] / 1001.9158506700721) / v[0])) / 0.0180152833;
    dhdt[2] = 0.0180152833 * (v[2] - v[1]);

    v[2] = (1001.9158506700721 * 0.0005 * sqrt(2. * g * (x[3] / 1001.9158506700721) / v[0])) / 0.0180152833;
    dhdt[3] = 0.0180152833 * (v[1] - v[2]);

    v[1] = (1001.9158506700721 * 0.0005 * sqrt(2. * g * (x[4] / 1001.9158506700721) / v[0])) / 0.0180152833;
    dhdt[4] = 0.0180152833 * (v[2] - v[1]);

    dhdt[5] = 0.0180152833 * (v[1] - (1001.9158506700721 * 0.0005 * sqrt(2. * g * (x[5] / 1001.9158506700721) / v[0])) / 0.0180152833);


    return dhdt;
}

#endif
