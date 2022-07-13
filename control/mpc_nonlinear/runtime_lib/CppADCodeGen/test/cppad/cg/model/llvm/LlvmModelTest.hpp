#ifndef CPPAD_CG_LLVMMODELTEST_INCLUDED
#define CPPAD_CG_LLVMMODELTEST_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
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
#include <cppad/cg/cppadcg.hpp>
#include <cppad/cg/model/llvm/llvm.hpp>

#include "CppADCGModelTest.hpp"

class LlvmModelTest : public CppAD::cg::CppADCGModelTest {
protected:
    std::vector<double> x;
    std::unique_ptr<CppAD::cg::LlvmModelLibrary<Base> > llvmModelLib;
    std::unique_ptr<CppAD::cg::GenericModel<Base> > model;
    std::unique_ptr<CppAD::ADFun<CppAD::cg::CG<Base> > > fun;
public:
    template<class T>
    CppAD::ADFun<T>* modelFunc(std::vector<CppAD::AD<T> >& x) {
        using namespace CppAD;
        using namespace std;

        assert(x.size() == 3);

        CppAD::Independent(x);

        std::vector<CppAD::AD<T> > y(2);
        y[0] = CppAD::abs(x[0]) * x[1];
        y[1] = CppAD::cos(x[2]);

        // f(v) = |w|
        return new CppAD::ADFun<T>(x, y);
    }

    virtual std::unique_ptr<CppAD::cg::LlvmModelLibrary<Base> >
    compileLib(CppAD::cg::LlvmModelLibraryProcessor<double>& p) = 0;

    void SetUp() override {
        using namespace CppAD;
        using namespace CppAD::cg;

        x = {-1, 2, 3};

        std::vector<AD<CG<double> > > u(3);
        //u[0] = x[0];

        fun.reset(modelFunc<CG<Base> >(u));

        /**
         * Create the dynamic library
         * (generate and compile source code)
         */
        ModelCSourceGen<double> modelSrcGen(*fun, "mySmallModel");
        modelSrcGen.setCreateForwardZero(true);
        modelSrcGen.setCreateJacobian(true);
        modelSrcGen.setCreateHessian(true);
        modelSrcGen.setCreateSparseJacobian(true);
        modelSrcGen.setCreateSparseHessian(true);
        modelSrcGen.setCreateForwardOne(true);
        modelSrcGen.setMultiThreading(false);

        ModelLibraryCSourceGen<double> libSrcGen(modelSrcGen);
        libSrcGen.setVerbose(this->verbose_);
        libSrcGen.setMultiThreading(MultiThreadingType::NONE);

        LlvmModelLibraryProcessor<double> p(libSrcGen);

        llvmModelLib = compileLib(p);
        model = llvmModelLib->model("mySmallModel");
        ASSERT_TRUE(model != nullptr);
    }

    void TearDown() override {
        fun.reset();
        model.reset(); // must be freed before llvm_shutdown()
        llvmModelLib.reset(); // must be freed before llvm_shutdown()

        CppAD::cg::CppADCGModelTest::TearDown();
    }

    // Per-test-case tear-down.
    // Called after the last test in this test case.
    static void TearDownTestCase() {
        // llvm_shutdown must be called once for all tests!
        llvm::llvm_shutdown();
    }
};

#endif //CPPAD_CG_LLVMMODELTEST_INCLUDED
