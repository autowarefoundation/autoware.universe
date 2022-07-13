#ifndef CPPAD_CG_TEST_CPPADCGDYNAMICTEST_INCLUDED
#define CPPAD_CG_TEST_CPPADCGDYNAMICTEST_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
 *    Copyright (C) 2012 Ciengis
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
#include "CppADCGModelTest.hpp"
#include "gccCompilerFlags.hpp"

#ifdef CPPAD_CG_SYSTEM_LINUX
#include <dlfcn.h>
#endif

namespace CppAD {
namespace cg {

class CppADCGDynamicTest : public CppADCGModelTest {
public:
    using CGD = CG<double>;
    using ADCG = AD<CGD>;
protected:
    const std::string _name;
    bool _denseJacobian;
    bool _denseHessian;
    bool _forwardOne;
    bool _reverseOne;
    bool _reverseTwo;
    MultiThreadingType _multithread;
    bool _multithreadDisabled;
    ThreadPoolScheduleStrategy _multithreadScheduler;
    std::vector<Base> _xTape;
    std::vector<double> _xRun;
    size_t _maxAssignPerFunc = 100;
    double epsilonR = 1e-14;
    double epsilonA = 1e-14;
    std::vector<double> _xNorm;
    std::vector<double> _eqNorm;
    std::unique_ptr<ADFun<CGD>> _fun;
    std::unique_ptr<DynamicLib<double>> _dynamicLib;
    std::unique_ptr<GenericModel<double>> _model;
    std::vector<size_t> _jacRow;
    std::vector<size_t> _jacCol;
    std::vector<size_t> _hessRow;
    std::vector<size_t> _hessCol;
public:

    explicit CppADCGDynamicTest(std::string testName,
                                bool verbose = false,
                                bool printValues = false) :
            CppADCGModelTest(verbose, printValues),
            _name(std::move(testName)),
            _denseJacobian(true),
            _denseHessian(true),
            _forwardOne(true),
            _reverseOne(true),
            _reverseTwo(true),
            _multithread(MultiThreadingType::NONE),
            _multithreadDisabled(false),
            _multithreadScheduler(ThreadPoolScheduleStrategy::DYNAMIC) {
    }

    virtual std::vector<ADCGD> model(const std::vector<ADCGD>& ind) = 0;

    void SetUp() override {
        ASSERT_EQ(_xTape.size(), _xRun.size());
        ASSERT_TRUE(_xNorm.empty() || _xRun.size() == _xNorm.size());

        using namespace std;

        // use a special object for source code generation
        std::vector<ADCGD> xTape(_xTape.size());
        for (size_t i = 0; i < xTape.size(); ++i) xTape[i] = _xTape[i];

        size_t abort_op_index = 0;
        bool record_compare = false;
        CppAD::Independent(xTape, abort_op_index, record_compare);


        if (!_xNorm.empty()) {
            for (size_t i = 0; i < xTape.size(); i++)
                xTape[i] *= _xNorm[i];
        }

        // dependent variable vector
        std::vector<ADCG> Z = model(xTape);

        if (!_eqNorm.empty()) {
            ASSERT_EQ(Z.size(), _eqNorm.size());
            for (size_t i = 0; i < Z.size(); i++)
                Z[i] /= _eqNorm[i];
        }

        /**
         * create the CppAD tape as usual
         */
        // create f: U -> Z and vectors used for derivative calculations
        _fun.reset(new ADFun<CGD>());
        _fun->Dependent(Z);

        /**
         * Create the dynamic library
         * (generate and compile source code)
         */
        ModelCSourceGen<double> modelSourceGen(*_fun, _name + "dynamic");

        modelSourceGen.setCreateForwardZero(true);
        modelSourceGen.setCreateJacobian(_denseJacobian);
        modelSourceGen.setCreateHessian(_denseHessian);
        modelSourceGen.setCreateSparseJacobian(true);
        modelSourceGen.setCreateSparseHessian(true);
        modelSourceGen.setCreateForwardOne(_forwardOne);
        modelSourceGen.setCreateReverseOne(_reverseOne);
        modelSourceGen.setCreateReverseTwo(_reverseTwo);
        modelSourceGen.setMaxAssignmentsPerFunc(_maxAssignPerFunc);
        modelSourceGen.setMultiThreading(true);

        if (!_jacRow.empty())
            modelSourceGen.setCustomSparseJacobianElements(_jacRow, _jacCol);

        if (!_hessRow.empty())
            modelSourceGen.setCustomSparseHessianElements(_hessRow, _hessCol);

        ModelLibraryCSourceGen<double> libSourceGen(modelSourceGen);
        libSourceGen.setMultiThreading(_multithread);

        SaveFilesModelLibraryProcessor<double>::saveLibrarySourcesTo(libSourceGen, "sources_" + _name + "_1");

        DynamicModelLibraryProcessor<double> p(libSourceGen);

        // some additional tests
        ASSERT_EQ(p.getLibraryName(), "cppad_cg_model");
        p.setLibraryName("cppad_cg_lib");
        ASSERT_EQ(p.getLibraryName(), "cppad_cg_lib");

        ASSERT_EQ(p.getCustomLibraryExtension(), nullptr);
        p.setCustomLibraryExtension(".so.123");
        ASSERT_NE(p.getCustomLibraryExtension(), nullptr);
        ASSERT_EQ(*p.getCustomLibraryExtension(), ".so.123");

        p.removeCustomLibraryExtension();
        ASSERT_EQ(p.getCustomLibraryExtension(), nullptr);

        const auto& cp = p;
        ASSERT_TRUE(cp.getOptions().empty());

        GccCompiler<double> compiler;
        //compiler.setSaveToDiskFirst(true); // useful to detect problem
        prepareTestCompilerFlags(compiler);
        if(libSourceGen.getMultiThreading() == MultiThreadingType::OPENMP) {
            compiler.addCompileFlag("-fopenmp");
            compiler.addCompileFlag("-pthread");
            compiler.addCompileLibFlag("-fopenmp");

#ifdef CPPAD_CG_SYSTEM_LINUX
            // this is required because the OpenMP implementation in GCC causes a segmentation fault on dlclose
            p.getOptions()["dlOpenMode"] = std::to_string(RTLD_NOW | RTLD_NODELETE);
            ASSERT_TRUE(!p.getOptions().empty());
#endif
        } else if(libSourceGen.getMultiThreading() == MultiThreadingType::PTHREADS) {
            compiler.addCompileFlag("-pthread");
        }

        _dynamicLib = p.createDynamicLibrary(compiler);
        _dynamicLib->setThreadPoolVerbose(this->verbose_);
        _dynamicLib->setThreadNumber(2);
        _dynamicLib->setThreadPoolDisabled(_multithreadDisabled);
        _dynamicLib->setThreadPoolSchedulerStrategy(_multithreadScheduler);
        _dynamicLib->setThreadPoolGuidedMaxWork(0.75);

        /**
         * test the library
         */
        _model = _dynamicLib->model(_name + "dynamic");
        ASSERT_TRUE(_model != nullptr);
    }

    void TearDown() override {
        _fun.reset();
    }

    void testForwardZero() {
        this->testForwardZeroResults(*_model, *_fun, nullptr, _xRun, epsilonR, epsilonA);
    }

    // Jacobian
    void testDenseJacobian () {
        this->testDenseJacResults(*_model, *_fun, _xRun, epsilonR, epsilonA);
    }

    void testDenseHessian() {
        this->testDenseHessianResults(*_model, *_fun, _xRun, epsilonR, epsilonA);
    }

    // sparse Jacobian
    void testJacobian() {
        // sparse Jacobian again (make sure the second run is also OK)
        size_t n_tests = _dynamicLib->getThreadNumber() > 1 ? 2 : 1;

        this->testSparseJacobianResults(n_tests, *_model, *_fun, nullptr, _xRun, !_jacRow.empty(), epsilonR,
                                        epsilonA);
    }

    // sparse Hessian
    void testHessian() {
        // sparse Hessian again (make sure the second run is also OK)
        size_t n_tests = _dynamicLib->getThreadNumber() > 1 ? 2 : 1;

        this->testSparseHessianResults(n_tests, *_model, *_fun, nullptr, _xRun, !_hessRow.empty(), epsilonR,
                                       epsilonA);
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
