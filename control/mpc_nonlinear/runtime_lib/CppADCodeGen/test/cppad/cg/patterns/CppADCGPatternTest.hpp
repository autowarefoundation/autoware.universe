#ifndef CPPAD_CG_TEST_CPPADCGPATTERNTEST_INCLUDED
#define	CPPAD_CG_TEST_CPPADCGPATTERNTEST_INCLUDED
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
#include "CppADCGTest.hpp"
#include "gccCompilerFlags.hpp"
#include "pattern_test_model.hpp"

namespace CppAD {
namespace cg {

class CppADCGPatternTest : public CppADCGTest {
public:
    using Base = double;
    using CGD = CppAD::cg::CG<Base>;
    using ADCGD = CppAD::AD<CGD>;
protected:
    PatternTestModel<CG<Base> >* model_;
    bool testZeroOrder_;
    bool testJacobian_;
    bool testHessian_;
    std::vector<Base> xNorm_;
    std::vector<Base> eqNorm_;
    std::vector<atomic_base<Base>*> atoms_;
    Base epsilonA_;
    Base epsilonR_;
    Base hessianEpsilonA_;
    Base hessianEpsilonR_;
    std::vector<std::set<size_t> > customJacSparsity_;
    std::vector<std::set<size_t> > customHessSparsity_;
private:
    std::unique_ptr<DefaultPatternTestModel<CG<Base> > > modelMem_;
public:

    inline CppADCGPatternTest(bool verbose = false, bool printValues = false) :
        CppADCGTest(verbose, printValues),
        model_(nullptr),
        testZeroOrder_(true),
        testJacobian_(true),
        testHessian_(true),
        epsilonA_(std::numeric_limits<Base>::epsilon() * 1e2),
        epsilonR_(std::numeric_limits<Base>::epsilon() * 1e2),
        hessianEpsilonA_(std::numeric_limits<Base>::epsilon() * 1e2),
        hessianEpsilonR_(std::numeric_limits<Base>::epsilon() * 1e2) {
        //this->verbose_ = true;
    }

    void TearDown() override {
        modelMem_.reset(nullptr);

        CppADCGTest::TearDown();
    }

    void setModel(PatternTestModel<CG<Base> >& model) {
        model_ = &model;
    }

    void setModel(std::vector<ADCGD> (*model)(const std::vector<ADCGD>& x, size_t repeat)) {
        modelMem_.reset(new DefaultPatternTestModel<CG<Base> >(model));
        setModel(*modelMem_);
    }

    void testPatternDetection(size_t m,
                              size_t n,
                              size_t repeat) {
        testPatternDetection(m, n, repeat, 1);
    }

    void testPatternDetection(size_t m,
                              size_t n,
                              size_t repeat,
                              size_t n_loops,
                              long commonVars = -1) {
        std::vector<std::vector<std::set<size_t> > > loops(n_loops);
        testPatternDetection(m, n, repeat, loops, commonVars);
    }

    void testPatternDetection(size_t m,
                              size_t n,
                              size_t repeat,
                              const std::vector<std::vector<std::set<size_t> > >& loops,
                              long commonVars = -1) {
        //size_t m2 = repeat * m;
        size_t n2 = repeat * n;

        /**
         * Tape model
         */
        std::vector<Base> xb(n2);
        for (size_t j = 0; j < n2; j++)
            xb[j] = 0.5;

        testPatternDetection(m, xb, repeat, loops, commonVars);
    }

    void testPatternDetection(size_t m,
                              const std::vector<Base>& xb,
                              size_t repeat,
                              const std::vector<std::vector<std::set<size_t> > >& loops = std::vector<std::vector<std::set<size_t> > >(1),
                              long commonVars = -1) {
        std::vector<std::set<size_t> > depCandidates = createRelatedDepCandidates(m, repeat);

        testPatternDetection(xb, repeat, depCandidates, loops, commonVars);
    }

    void testPatternDetection(const std::vector<Base>& xb,
                              size_t repeat,
                              const std::vector<std::set<size_t> >& relatedDepCandidates,
                              const std::vector<std::vector<std::set<size_t> > >& loops = std::vector<std::vector<std::set<size_t> > >(1),
                              long commonVars = -1) {
        using namespace CppAD;

        assert(model_ != nullptr);

        /**
         * Tape model
         */
        std::unique_ptr<ADFun<CGD> > fun(tapeModel(repeat, xb));

        testPatternDetectionResults(*fun, repeat, relatedDepCandidates, loops, commonVars);
    }

    void testLibCreation(const std::string& libName,
                         size_t m,
                         size_t n,
                         size_t repeat,
                         size_t mExtra = 0) {
        size_t n2 = repeat * n;
        std::vector<Base> x(n2);
        for (size_t j = 0; j < n2; j++)
            x[j] = 0.5 * (j + 1);

        testLibCreation(libName, m, repeat, x);
    }

    void testLibCreation(const std::string& libName,
                         size_t m,
                         size_t repeat,
                         const std::vector<Base>& xb) {

        std::vector<std::set<size_t> > relatedDepCandidates = createRelatedDepCandidates(m, repeat);

        testLibCreation(libName, relatedDepCandidates, repeat, xb);
    }

    void testLibCreation(const std::string& libName,
                         const std::vector<std::set<size_t> >& relatedDepCandidates,
                         size_t repeat,
                         const std::vector<Base>& xb) {
        using namespace CppAD;

        assert(!relatedDepCandidates.empty());
        assert(model_ != nullptr);

        /**
         * Tape model
         */
        std::unique_ptr<ADFun<CGD> > fun(tapeModel(repeat, xb));

        defineCustomSparsity(*fun);

        testSourceCodeGen(*fun, relatedDepCandidates, libName, xb, JacobianADMode::Forward, testJacobian_, testHessian_);
        if (testJacobian_) {
            testSourceCodeGen(*fun, relatedDepCandidates, libName, xb, JacobianADMode::Forward, true, false, true);
            testSourceCodeGen(*fun, relatedDepCandidates, libName, xb, JacobianADMode::Reverse, true, false);
            testSourceCodeGen(*fun, relatedDepCandidates, libName, xb, JacobianADMode::Reverse, true, false, true);
        }

        if (testHessian_) {
            testSourceCodeGen(*fun, relatedDepCandidates, libName, xb, JacobianADMode::Forward, false, true, false, true);
        }

    }

    ADFun<CGD>* tapeModel(size_t repeat,
                          const std::vector<Base>& xb) {
        /**
         * Tape model
         */
        std::vector<ADCGD> x(xb.size());
        for (size_t j = 0; j < xb.size(); j++)
            x[j] = xb[j];
        CppAD::Independent(x);
        if (!xNorm_.empty()) {
            assert(x.size() == xNorm_.size());
            for (size_t j = 0; j < x.size(); j++)
                x[j] *= xNorm_[j];
        }

        std::vector<ADCGD> y = model_->evaluateModel(x, repeat);
        if (!eqNorm_.empty()) {
            assert(y.size() == eqNorm_.size());
            for (size_t i = 0; i < y.size(); i++)
                y[i] /= eqNorm_[i];
        }

        std::unique_ptr<ADFun<CGD> > fun(new ADFun<CGD>());
        fun->Dependent(y);
        return fun.release();
    }

    std::vector<std::set<size_t> > createRelatedDepCandidates(size_t m,
                                                              size_t repeat) {
        std::vector<std::set<size_t> > relatedDepCandidates(m);
        for (size_t i = 0; i < repeat; i++) {
            for (size_t ii = 0; ii < m; ii++) {
                relatedDepCandidates[ii].insert(i * m + ii);
            }
        }
        return relatedDepCandidates;
    }

    void testPatternDetectionResults(ADFun<CGD>& fun,
                                     size_t m,
                                     size_t repeat,
                                     const std::vector<std::vector<std::set<size_t> > >& loops,
                                     long commonVars = -1) {
        std::vector<std::set<size_t> > depCandidates = createRelatedDepCandidates(m, repeat);

        testPatternDetectionResults(fun, repeat, depCandidates, loops, commonVars);
    }

    void testPatternDetectionResults(ADFun<CGD>& fun,
                                     size_t repeat,
                                     const std::vector<std::set<size_t> >& depCandidates,
                                     const std::vector<std::vector<std::set<size_t> > >& loops,
                                     long commonVars = -1) {
        using namespace std;

        /**
         * Generate operation graph
         */
        CodeHandler<double> h;
        size_t n2 = fun.Domain();

        std::vector<CGD> xx(n2);
        h.makeVariables(xx);
        for (size_t j = 0; j < n2; j++) {
            xx[j].setValue(j);
        }

        std::vector<CGD> yy = fun.Forward(0, xx);

        DependentPatternMatcher<double> matcher(depCandidates, yy, xx);

        LoopFreeModel<Base>* nonLoopTape;
        SmartSetPointer<LoopModel<Base> > loopTapes;
        matcher.generateTapes(nonLoopTape, loopTapes.s);

        if (commonVars >= 0) {
            ASSERT_EQ(nonLoopTape->getTapeDependentCount(), (size_t) commonVars);
        }

        delete nonLoopTape;

        //std::cout << "loops: " << matcher.getLoops().size() << std::endl;
        ASSERT_EQ(loopTapes.size(), loops.size());

        //std::cout << "equation patterns: " << matcher.getEquationPatterns().size() << std::endl;

        /**
         * order loops and equation patterns by the lowest used dependent
         */
        //  - calculated
        map<size_t, map<size_t, set<size_t> > > orderedCalcLoops;

        const std::vector<Loop<Base>*>& calcLoops = matcher.getLoops();
        for (auto loop : calcLoops) {
            size_t minDep = (std::numeric_limits<size_t>::max)();

            map<size_t, set<size_t> > dependents;
            set<EquationPattern<Base>*>::const_iterator iteq;
            for (iteq = loop->equations.begin(); iteq != loop->equations.end(); ++iteq) {
                EquationPattern<Base>* eq = *iteq;
                size_t minEqDep = *eq->dependents.begin();
                minDep = std::min<size_t>(minDep, *eq->dependents.begin());
                dependents[minEqDep] = eq->dependents;
            }

            orderedCalcLoops[minDep] = dependents;
        }

        //print(orderedCalcLoops);

        //  - expected
        bool defined = false;
        map<size_t, map<size_t, set<size_t> > > orderedExpectedLoops;
        for (const auto& eqPatterns : loops) {
            if (!eqPatterns.empty()) {
                defined = true;
                /**
                 * check every equation
                 */
                size_t minDep = (std::numeric_limits<size_t>::max)();

                map<size_t, set<size_t> > dependents;
                for (const auto& eqPattern : eqPatterns) {
                    size_t minEqDep = *eqPattern.begin();
                    minDep = std::min<size_t>(minDep, *eqPattern.begin());
                    dependents[minEqDep] = eqPattern;
                }
                orderedExpectedLoops[minDep] = dependents;
            }
        }

        if (defined) {
            auto itLexp = orderedExpectedLoops.begin();
            auto itLcalc = orderedCalcLoops.begin();
            for (; itLexp != orderedExpectedLoops.end(); ++itLexp, ++itLcalc) {
                ASSERT_EQ(itLexp->first, itLcalc->first);
                ASSERT_EQ(itLexp->second.size(), itLcalc->second.size());

                auto itEexp = itLexp->second.begin();
                auto itEcalc = itLcalc->second.begin();
                for (; itEexp != itLexp->second.end(); ++itEexp, ++itEcalc) {
                    ASSERT_EQ(itEexp->first, itEcalc->first);
                    ASSERT_TRUE(itEexp->second == itEcalc->second);
                }
            }

        } else {
            ASSERT_EQ(matcher.getEquationPatterns().size(), depCandidates.size());
            for (auto eqp : matcher.getEquationPatterns()) {
                ASSERT_EQ(eqp->dependents.size(), repeat);
            }
        }
    }

    void testSourceCodeGen(ADFun<CGD>& fun,
                           size_t m, size_t repeat,
                           size_t mExtra,
                           const std::string& name,
                           const std::vector<Base>& xTypical,
                           JacobianADMode jacMode,
                           bool jacobian = true,
                           bool hessian = true,
                           bool forReverseOne = false,
                           bool reverseTwo = false) {

        std::vector<std::set<size_t> > relatedDepCandidates = createRelatedDepCandidates(m, repeat);

        testSourceCodeGen(fun, relatedDepCandidates, name, xTypical,
                          jacMode, jacobian, hessian, forReverseOne, reverseTwo);
    }

    void testSourceCodeGen(ADFun<CGD>& fun,
                           const std::vector<std::set<size_t> >& relatedDepCandidates,
                           const std::string& name,
                           const std::vector<Base>& xTypical,
                           JacobianADMode jacMode,
                           bool jacobian = true,
                           bool hessian = true,
                           bool forRevOne = false,
                           bool reverseTwo = false) {

        bool loadModels = this->testZeroOrder_ || jacobian || hessian;

        std::string libBaseName = name;
        if (jacobian) {
            if (!forRevOne) libBaseName += "d";
            if (jacMode == JacobianADMode::Forward) libBaseName += "F";
            else if (jacMode == JacobianADMode::Reverse) libBaseName += "R";
        }
        if (hessian && reverseTwo)
            libBaseName += "rev2";


        assert(fun.Domain() == xTypical.size());
        /**
         * Create the dynamic library
         * (generate and compile source code)
         */
        ModelCSourceGen<double> compHelpL(fun, libBaseName + "Loops");
        compHelpL.setCreateForwardZero(true);
        compHelpL.setJacobianADMode(jacMode);
        compHelpL.setCreateJacobian(false);
        compHelpL.setCreateHessian(false);
        compHelpL.setCreateSparseJacobian(jacobian);
        compHelpL.setCreateSparseHessian(hessian);
        compHelpL.setCreateForwardOne(forRevOne && jacMode == JacobianADMode::Forward);
        compHelpL.setCreateReverseOne(forRevOne && jacMode == JacobianADMode::Reverse);
        compHelpL.setCreateReverseTwo(reverseTwo);
        //compHelpL.setMaxAssignmentsPerFunc(maxAssignPerFunc);
        compHelpL.setRelatedDependents(relatedDepCandidates);
        compHelpL.setTypicalIndependentValues(xTypical);
        compHelpL.setParameterPrecision(std::numeric_limits<Base>::digits10 + 4);

        if (!customJacSparsity_.empty())
            compHelpL.setCustomSparseJacobianElements(customJacSparsity_);

        if (!customHessSparsity_.empty())
            compHelpL.setCustomSparseHessianElements(customHessSparsity_);

        GccCompiler<double> compiler;
        prepareTestCompilerFlags(compiler);
        compiler.setSourcesFolder("sources_" + libBaseName);
        compiler.setSaveToDiskFirst(true);

        ModelLibraryCSourceGen<double> compDynHelpL(compHelpL);
        compDynHelpL.setVerbose(this->verbose_);

        //SaveFilesModelLibraryProcessor<double>::saveLibrarySourcesTo(compDynHelpL, "sources_" + libBaseName);

        DynamicModelLibraryProcessor<double> p(compDynHelpL, libBaseName + "Loops");
        std::unique_ptr<DynamicLib<double> > dynamicLibL = p.createDynamicLibrary(compiler);
        std::unique_ptr<GenericModel<double> > modelL;
        if (loadModels) {
            modelL = dynamicLibL->model(libBaseName + "Loops");
            ASSERT_TRUE(modelL != nullptr);
            for (auto& atom : atoms_)
                modelL->addAtomicFunction(*atom);
        }
        /**
         * Without the loops
         */
        ModelCSourceGen<double> compHelp(fun, libBaseName + "NoLoops");
        compHelp.setCreateForwardZero(testZeroOrder_);
        compHelp.setJacobianADMode(jacMode);
        compHelp.setCreateJacobian(false);
        compHelp.setCreateHessian(false);
        compHelp.setCreateSparseJacobian(jacobian);
        compHelp.setCreateSparseHessian(hessian);
        compHelp.setCreateForwardOne(false);
        compHelp.setCreateReverseOne(false);
        compHelp.setCreateReverseTwo(reverseTwo);
        compHelp.setTypicalIndependentValues(xTypical);
        compHelp.setParameterPrecision(std::numeric_limits<Base>::digits10 + 4);
        //compHelp.setMaxAssignmentsPerFunc(maxAssignPerFunc);

        if (!customJacSparsity_.empty())
            compHelp.setCustomSparseJacobianElements(customJacSparsity_);

        if (!customHessSparsity_.empty())
            compHelp.setCustomSparseHessianElements(customHessSparsity_);

        ModelLibraryCSourceGen<double> compDynHelp(compHelp);
        compDynHelp.setVerbose(this->verbose_);

        SaveFilesModelLibraryProcessor<double>::saveLibrarySourcesTo(compDynHelp, "sources_" + libBaseName);

        DynamicModelLibraryProcessor<double> p2(compDynHelp, libBaseName + "NoLoops");
        std::unique_ptr<DynamicLib<double> > dynamicLib = p2.createDynamicLibrary(compiler);

        /**
         * reference library
         */
        std::unique_ptr<GenericModel<double> > model;
        if (loadModels) {
            model = dynamicLib->model(libBaseName + "NoLoops");
            for (auto& atom : atoms_)
                model->addAtomicFunction(*atom);
        }

        if (!loadModels)
            return;

        /**
         * Compare results
         */
        ASSERT_EQ(modelL->Domain(), model->Domain());
        ASSERT_EQ(modelL->Range(), model->Range());

        std::vector<double> x = xTypical;

        // test model (zero-order)
        if (compHelp.isCreateForwardZero()) {
            std::vector<double> yl = modelL->ForwardZero(x);
            std::vector<double> y = model->ForwardZero(x);
            ASSERT_TRUE(compareValues(yl, y, epsilonR_, epsilonA_));
        }

        // test Jacobian
        if (compHelp.isCreateSparseJacobian()) {
            compareVectorSetValues(modelL->JacobianSparsitySet(),
                                   model->JacobianSparsitySet());

            std::vector<double> jacl, jac;
            std::vector<size_t> rowsl, colsl, rows, cols;
            modelL->SparseJacobian(x, jacl, rowsl, colsl);
            model->SparseJacobian(x, jac, rows, cols);

            ASSERT_TRUE(compareValues(jacl, jac, epsilonR_, epsilonA_));
        }

        // test Hessian
        if (compHelp.isCreateSparseHessian()) {
            compareVectorSetValues(modelL->HessianSparsitySet(),
                                   model->HessianSparsitySet());

            std::vector<double> w(fun.Range());
            for (size_t i = 0; i < w.size(); i++) {
                w[i] = 0.5 * (i + 1);
            }
            std::vector<double> hessl, hess;
            std::vector<size_t> rowsl, colsl, rows, cols;
            modelL->SparseHessian(x, w, hessl, rowsl, colsl);
            model->SparseHessian(x, w, hess, rows, cols);

            ASSERT_TRUE(compareValues(hessl, hess, hessianEpsilonR_, hessianEpsilonA_));
        }

    }
protected:

    inline virtual void defineCustomSparsity(ADFun<CGD>& fun) {
    }
};

} // END cg namespace
} // END CppAD namespace

#endif
