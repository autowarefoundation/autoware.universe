/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
 *    Copyright (C) 2019 Joao Leal
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

#include <dlfcn.h>
#include <cppad/cg/cppadcg.hpp>

#include "CppADCGTest.hpp"
#include "TestException.hpp"

#ifndef CPPAD_CG_CPPADCGOPERATIONTEST_HPP
#define	CPPAD_CG_CPPADCGOPERATIONTEST_HPP

namespace CppAD {
namespace cg {

class CppADCGOperationTest : public CppADCGTest {
public:

    inline explicit CppADCGOperationTest(bool verbose = false,
                                         bool printValues = false) :
        CppADCGTest(verbose, printValues) {
    }

    void TearDown() override {
        ASSERT_FALSE(CppAD::memory_leak());
    }

protected:
    /**
     *
     * @throws TestException
     */
    void* loadLibrary(const std::string& library);

    /**
     *
     * @throws TestException
     */
    void* getFunction(void * libHandle, const std::string& functionName);

    void closeLibrary(void* libHandle);

    /**
     *
     * @throws TestException
     */
    void compile(const std::string& source, const std::string& library);

    std::vector<std::vector<double> > runDefault0(CppAD::ADFun<double>& f,
                                                  const std::vector<std::vector<double> >& ind);

    std::vector<std::vector<double> > runDefault0(CppAD::ADFun<double>& f,
                                                  const std::vector<std::vector<double> >& indV,
                                                  int& comparisons);

    std::vector<std::vector<double> > run0(CppAD::ADFun<CppAD::cg::CG<double> >& f,
                                           const std::string& library, const std::string& function,
                                           const std::vector<std::vector<double> >& ind);

    std::vector<std::vector<double> > run0(CppAD::ADFun<CppAD::cg::CG<double> >& f,
                                           const std::string& library, const std::string& function,
                                           const std::vector<std::vector<double> >& indV,
                                           int& comparisons);

    std::vector<std::vector<double> > run0TapeWithValues(CppAD::ADFun<CppAD::cg::CG<double> >& f,
                                                         const std::vector<std::vector<double> >& indV);

    std::vector<std::vector<double> > runSparseJacDefault(CppAD::ADFun<double>& f,
                                                          const std::vector<std::vector<double> >& ind);

    std::vector<std::vector<double> > runSparseJac(CppAD::ADFun<CppAD::cg::CG<double> >& f,
                                                   const std::string& library,
                                                   const std::string& functionJac,
                                                   const std::vector<std::vector<double> >& indV);

    void test0(const std::string& test,
               CppAD::ADFun<double>* (*func1)(const std::vector<CppAD::AD<double> >&),
               CppAD::ADFun<CppAD::cg::CG<double> >* (*func2)(const std::vector<CppAD::AD<CppAD::cg::CG<double> > >&),
               const std::vector<std::vector<double> >& indV,
               int& comparisons,
               double epsilonR = 1e-14, double epsilonA = 1e-14);

    void test0nJac(const std::string& test,
                   CppAD::ADFun<double>* (*func1)(const std::vector<CppAD::AD<double> >&),
                   CppAD::ADFun<CppAD::cg::CG<double> >* (*func2)(const std::vector<CppAD::AD<CppAD::cg::CG<double> > >&),
                   const std::vector<double>& ind,
                   double epsilonR = 1e-14, double epsilonA = 1e-14);

    void test0nJac(const std::string& test,
                   CppAD::ADFun<double>* (*func1)(const std::vector<CppAD::AD<double> >&),
                   CppAD::ADFun<CppAD::cg::CG<double> >* (*func2)(const std::vector<CppAD::AD<CppAD::cg::CG<double> > >&),
                   const std::vector<std::vector<double> >& indV,
                   double epsilonR = 1e-14, double epsilonA = 1e-14);

};

} // END cg namespace
} // END CppAD namespace

#include "CppADCGOperationTestImpl.hpp"

#endif
