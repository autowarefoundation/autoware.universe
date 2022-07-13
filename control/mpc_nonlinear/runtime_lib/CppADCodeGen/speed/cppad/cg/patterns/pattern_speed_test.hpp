#ifndef CPPAD_CG_PATTERNSPEEDTEST_INCLUDED
#define	CPPAD_CG_PATTERNSPEEDTEST_INCLUDED
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

#include <cppad/cg/cppadcg.hpp>
#include <cppad/cg/model/llvm/llvm.hpp>
#include "job_speed_listener.hpp"

namespace CppAD {
namespace cg {

class PatternSpeedTest {
public:
    using Base = double;
    using CGD = CppAD::cg::CG<Base>;
    using ADCGD = CppAD::AD<CGD>;
    using duration = std::chrono::steady_clock::duration;
private:

    /**
     *
     */
    template<class T>
    class Model {
    protected:
        PatternSpeedTest& self_;
    public:

        Model(PatternSpeedTest& s) : self_(s) {
        }

        virtual std::vector<AD<T> > evaluate(const std::vector<AD<T> >& x, size_t repeat) = 0;
    };

    /**
     *
     */
    class ModelCppAD : public Model<Base> {
    public:

        ModelCppAD(PatternSpeedTest& s) : Model(s) {
        }

        virtual std::vector<AD<Base> > evaluate(const std::vector<AD<Base> >& x, size_t repeat) {
            return self_.modelCppAD(x, repeat);
        }
    };

    /**
     *
     */
    class ModelCppADCG : public Model<CG<Base> > {
    public:

        ModelCppADCG(PatternSpeedTest& s) : Model(s) {
        }

        virtual std::vector<AD<CG<Base> > > evaluate(const std::vector<AD<CG<Base> > >& x, size_t repeat) {
            return self_.modelCppADCG(x, repeat);
        }
    };
public:
    bool preparation;
    bool zeroOrder;
    bool sparseJacobian;
    bool sparseHessian;
    bool cppAD;
    bool cppADCG;
    bool cppADCGLoops;
    bool cppADCGLoopsLlvm;
protected:
    std::string libName_;
    bool testJacobian_;
    bool testHessian_;
    std::vector<Base> xNorm_;
    std::vector<Base> eqNorm_;
    std::vector<std::set<size_t> > customJacSparsity_;
    std::vector<std::set<size_t> > customHessSparsity_;
    std::unique_ptr<DynamicLib<Base> > dynamicLib_;
    std::unique_ptr<LlvmModelLibrary<Base> > llvmLib_;
    std::unique_ptr<GenericModel<Base> > model_;
    std::vector<GenericModel<Base>*> externalModels_;
    std::vector<std::string> compileFlags_;
    std::unique_ptr<ModelCSourceGen<double> > modelSourceGen_;
    std::unique_ptr<ModelLibraryCSourceGen<double> > libSourceGen_;
    JobSpeedListener listener_;
    bool verbose_;
    size_t nTimes_;
private:
    std::vector<duration> patternDection_; /// pattern detection
    std::vector<duration> graphGen_; //
    std::vector<duration> srcCodeGen_; /// source code generation
    std::vector<duration> srcCodeComp_; /// source code compilation
    std::vector<duration> dynLibComp_; /// compilation of the dynamic library
    std::vector<duration> jit_; /// compilation of the dynamic library
    std::vector<duration> total_; /// total time
public:

    inline PatternSpeedTest(const std::string& libName,
                            bool verbose = false) :
        preparation(true),
        zeroOrder(true),
        sparseJacobian(true),
        sparseHessian(true),
        cppAD(true),
        cppADCG(true),
        cppADCGLoops(true),
        cppADCGLoopsLlvm(true),
        libName_(libName),
        testJacobian_(true),
        testHessian_(true),
        verbose_(verbose),
        nTimes_(50) {

    }

    inline void setNumberOfExecutions(size_t nTimes) {
        nTimes_ = nTimes;
    }

    inline void setCompileFlags(const std::vector<std::string>& compileFlags) {
        compileFlags_ = compileFlags;
    }

    virtual std::vector<ADCGD> modelCppADCG(const std::vector<ADCGD>& x, size_t repeat) = 0;

    virtual std::vector<AD<Base> > modelCppAD(const std::vector<AD<Base> >& x, size_t repeat) = 0;

    inline void setExternalModels(const std::vector<GenericModel<Base>*>& atoms) {
        externalModels_ = atoms;
    }

    inline void measureSpeed(size_t m,
                             size_t n,
                             size_t repeat) {
        size_t n2 = repeat * n;
        std::vector<Base> x(n2);
        for (size_t j = 0; j < n2; j++)
            x[j] = 0.5 * (j + 1);

        std::vector<std::set<size_t> > relatedDepCandidates = createRelatedDepCandidates(m, repeat);

        measureSpeed(relatedDepCandidates, repeat, x);
    }

    inline void measureSpeed(size_t m,
                             size_t repeat,
                             const std::vector<Base>& xb) {

        std::vector<std::set<size_t> > relatedDepCandidates = createRelatedDepCandidates(m, repeat);

        measureSpeed(relatedDepCandidates, repeat, xb);
    }

    inline void measureSpeed(const std::vector<std::set<size_t> >& relatedDepCandidates,
                             size_t repeat,
                             const std::vector<Base>& xb) {
        using namespace CppAD;

        std::cout << libName_ << "\n";
        std::cout << "n=" << repeat << "\n";
        std::cerr << libName_ << "\n";
        std::cerr << "n=" << repeat << "\n";

        /*******************************************************************
         * CppADCG (without Loops)
         ******************************************************************/
        measureSpeedCppADCG(repeat, xb);

        /*******************************************************************
         * CppADCG (Loops)
         ******************************************************************/
        measureSpeedCppADCGWithLoops(relatedDepCandidates, repeat, xb);

        measureSpeedCppADCGWithLoopsLlvm(relatedDepCandidates, repeat, xb);

        /*******************************************************************
         * CppAD
         ******************************************************************/
        measureSpeedCppAD(repeat, xb);
    }

    inline static size_t parseProgramArguments(int pos, int argc, char **argv, size_t defaultRepeat) {
        if (argc > pos) {
            std::istringstream is(argv[pos]);
            size_t repeat;
            is >> repeat;
            return repeat;
        }
        return defaultRepeat;
    }

protected:

    inline void measureSpeedCppADCG(size_t repeat,
                                    const std::vector<Base>& xb) {
        using namespace std::chrono;
        using namespace CppAD;

        /*******************************************************************
         * CppADCG (without Loops)
         ******************************************************************/
        std::string head = "\n"
                "********************************************************************************\n"
                "CppADCG (without Loops) GCC\n"
                "********************************************************************************\n";
        std::cout << head << std::endl;
        std::cerr << head << std::endl;

        printStatHeader();

        ModelCppADCG model(*this);

        std::unique_ptr<ADFun<CGD> > fun;

        /**
         * preparation
         */
        std::vector<duration> dt(cppADCG ? (preparation ? nTimes_ : 1) : 0);
        for (size_t i = 0; i < dt.size(); i++) {
            // tape
            auto t0 = steady_clock::now();
            fun.reset(tapeModel(model, xb, repeat));
            dt[i] = steady_clock::now() - t0;

            // create dynamic lib
            createDynamicLib(*fun.get(), std::vector<std::set<size_t> >(), xb, i == dt.size() - 1, JacobianADMode::Reverse, testJacobian_, testHessian_);
        }
        printStat("model tape", dt);
        printCGResults();

        /**
         * execution
         */
        // evaluation speed
        executionSpeedCppADCG(xb, cppADCG);
    }

    inline void measureSpeedCppADCGWithLoops(const std::vector<std::set<size_t> >& relatedDepCandidates,
                                             size_t repeat,
                                             const std::vector<Base>& xb) {
        using namespace CppAD;
        using namespace std::chrono;

        std::string head = "\n"
                "********************************************************************************\n"
                "CppADCG (with Loops) GCC\n"
                "********************************************************************************\n";
        std::cout << head << std::endl;
        std::cerr << head << std::endl;

        printStatHeader();

        ModelCppADCG model(*this);

        std::unique_ptr<ADFun<CGD> > fun;

        /**
         * preparation
         */
        std::vector<duration> dt(cppADCGLoops ? (preparation ? nTimes_ : 1) : 0);
        for (size_t i = 0; i < dt.size(); i++) {
            // tape
            auto t0 = steady_clock::now();
            fun.reset(tapeModel(model, xb, repeat));
            dt[i] = steady_clock::now() - t0;

            // create dynamic lib
            createDynamicLib(*fun.get(), relatedDepCandidates, xb, i == dt.size() - 1, JacobianADMode::Reverse, testJacobian_, testHessian_);
        }
        printStat("model tape", dt);

        printCGResults();

        /**
         * execution
         */
        // evaluation speed
        executionSpeedCppADCG(xb, cppADCGLoops);
    }

    inline void measureSpeedCppADCGWithLoopsLlvm(const std::vector<std::set<size_t> >& relatedDepCandidates,
                                                 size_t repeat,
                                                 const std::vector<Base>& xb) {
        using namespace CppAD;

        std::string head = "\n"
                "********************************************************************************\n"
                "CppADCG (with Loops) LLVM\n"
                "********************************************************************************\n";
        std::cout << head << std::endl;
        std::cerr << head << std::endl;

        printStatHeader();

        JacobianADMode jacMode = JacobianADMode::Reverse;
        bool forReverseOne = false;
        bool reverseTwo = false;

        bool withLoops = !relatedDepCandidates.empty();
        std::string libBaseName = createLibBaseName(jacMode, forReverseOne, reverseTwo);

        /**
         * preparation
         */
        //create source code
        if (modelSourceGen_.get() == nullptr) {
            listener_.reset();
            // tape
            ModelCppADCG model(*this);
            std::unique_ptr<ADFun<CGD> > fun;
            fun.reset(tapeModel(model, xb, repeat));

            createSource(libBaseName, *fun.get(), relatedDepCandidates, xb, jacMode, testJacobian_, testHessian_, forReverseOne, reverseTwo);
        }

        size_t nTimes = cppADCGLoopsLlvm ? (preparation ? nTimes_ : 1) : 0;
        for (size_t i = 0; i < nTimes; i++) {
            // prepare LLVM module
            createJitModelLib(libBaseName, xb, withLoops);
        }
        printCGResults();

        /**
         * execution
         */
        // evaluation speed
        executionSpeedCppADCG(xb, cppADCGLoopsLlvm);
    }

    inline void printCGResults() {
        // save results
        if (!patternDection_.empty())
            printStat("loop detection", patternDection_);
        if (!graphGen_.empty())
            printStat("(graph generation)", graphGen_);
        if (!srcCodeGen_.empty())
            printStat("code generation", srcCodeGen_);
        if (!srcCodeComp_.empty())
            printStat("code compilation", srcCodeComp_);
        if (!dynLibComp_.empty())
            printStat("dynamic library compilation", dynLibComp_);
        if (!jit_.empty())
            printStat("JIT library preparation", jit_);
        printStat("total", total_);

        patternDection_.clear();
        graphGen_.clear();
        srcCodeGen_.clear();
        srcCodeComp_.clear();
        dynLibComp_.clear();
        jit_.clear();
        total_.clear();
    }

    static void printStat(const std::string& title,
                          const std::vector<duration>& times) {
        std::cout << std::setw(30) << title << ": ";
        printStat(times);
        std::cout << std::endl;

        std::cerr << std::setw(30) << title << ": ";
        for (size_t i = 0; i < times.size(); i++)
            std::cerr << std::setw(12) << std::chrono::duration<double>(times[i]).count() << " ";
        std::cerr << std::endl;
    }

    static void printStatHeader() {
        std::cout << std::setw(30) << "" << "  "
                  << std::setw(12) << "mean" << " +- " << std::setw(12) << "stdDev"
                  << "      "
                  << std::setw(12) << "min" << "|--["
                  << std::setw(12) << "q25" << ", "
                  << std::setw(12) << "median" << ", "
                  << std::setw(12) << "q75" << "]--|"
                  << std::setw(12) << "max"
                  << std::endl;
    }

    static void printStat(const std::vector<duration>& dtimes) {
        std::vector<double> times(dtimes.size());
        for (size_t i = 0; i < times.size(); i++)
            times[i] = std::chrono::duration<double>(dtimes[i]).count();

        double min, q25, median, q75, max;
        if (times.empty()) {
            min = q25 = median = q75 = max = std::numeric_limits<double>::quiet_NaN();

        } else {

            std::vector<double> sorted = times;
            std::sort(sorted.begin(), sorted.end());

            size_t middle1 = sorted.size() / 2;
            if (sorted.size() % 2 == 1) {
                median = sorted[middle1];
                if (sorted.size() <= 3) {
                    q25 = sorted.front();
                    q75 = sorted.back();
                } else {
                    size_t s = middle1 / 2;
                    q25 = (sorted[s] + sorted[s - 1]) / 2;
                    q75 = (sorted[middle1 + s] + sorted[middle1 + s + 1]) / 2;
                }
            } else {
                median = (sorted[middle1] + sorted[middle1 - 1]) / 2;
                if (sorted.size() <= 2) {
                    q25 = sorted.front();
                    q75 = sorted.back();
                } else {
                    size_t s = (middle1 - 1) / 2;
                    q25 = (sorted[s] + sorted[s - 1]) / 2;
                    q75 = (sorted[middle1 + s] + sorted[middle1 + s + 1]) / 2;
                }
            }

            min = sorted.front();
            max = sorted.back();
        }

        std::cout << std::setw(12) << mean(times) << " +- " << std::setw(12) << stdDev(times)
                << "      "
                << std::setw(12) << min << "|--["
                << std::setw(12) << q25 << ", "
                << std::setw(12) << median << ", "
                << std::setw(12) << q75 << "]--|"
                << std::setw(12) << max;
    }

    inline void measureSpeedCppAD(size_t repeat,
                                  const std::vector<Base>& xb) {
        using namespace std::chrono;

        std::string head = "\n"
                "********************************************************************************\n"
                "CppAD\n"
                "********************************************************************************\n";
        std::cout << head << std::endl;
        std::cerr << head << std::endl;

        printStatHeader();

        ModelCppAD model(*this);

        std::unique_ptr<ADFun<Base> > fun;

        /**
         * preparation
         */
        size_t nTimes = cppAD && preparation ? nTimes_ : 1;
        std::vector<duration> dt1(nTimes), dt2(nTimes);
        for (size_t i = 0; i < nTimes; i++) {
            // tape
            auto t0 = steady_clock::now();
            fun.reset(tapeModel(model, xb, repeat));
            dt1[i] = steady_clock::now() - t0;

            // optimize tape
            t0 = steady_clock::now();
            fun->optimize();
            dt2[i] = steady_clock::now() - t0;
        }
        printStat("model tape", dt1);
        printStat("optimize tape", dt2);

        /**
         * execution
         */
        // evaluation speed
        executionSpeedCppAD(*fun.get(), xb);
    }

    template<class T>
    ADFun<T>* tapeModel(Model<T>& model,
                        const std::vector<Base>& xb,
                        size_t repeat) {
        /**
         * Tape model
         */
        std::vector<AD<T> > x(xb.size());
        for (size_t j = 0; j < xb.size(); j++)
            x[j] = xb[j];
        CppAD::Independent(x);
        if (xNorm_.size() > 0) {
            assert(x.size() == xNorm_.size());
            for (size_t j = 0; j < x.size(); j++)
                x[j] *= xNorm_[j];
        }

        std::vector<AD<T> > y = model.evaluate(x, repeat);
        if (eqNorm_.size() > 0) {
            assert(y.size() == eqNorm_.size());
            for (size_t i = 0; i < y.size(); i++)
                y[i] /= eqNorm_[i];
        }

        std::unique_ptr<ADFun<T> > fun(new ADFun<T>());
        fun->Dependent(y);

        return fun.release();
    }

    inline std::vector<std::set<size_t> > createRelatedDepCandidates(size_t m,
                                                                     size_t repeat) {
        std::vector<std::set<size_t> > relatedDepCandidates(m);
        for (size_t i = 0; i < repeat; i++) {
            for (size_t ii = 0; ii < m; ii++) {
                relatedDepCandidates[ii].insert(i * m + ii);
            }
        }
        return relatedDepCandidates;
    }

    inline std::string createLibBaseName(JacobianADMode jacMode,
                                         bool jacobian = true,
                                         bool hessian = true,
                                         bool forReverseOne = false,
                                         bool reverseTwo = false) const {
        std::string libBaseName = libName_;
        if (jacobian) {
            if (!forReverseOne) libBaseName += "d";
            if (jacMode == JacobianADMode::Forward) libBaseName += "F";
            else if (jacMode == JacobianADMode::Reverse) libBaseName += "R";
        }
        if (hessian && reverseTwo)
            libBaseName += "rev2";
        return libBaseName;
    }

    inline void createSource(const std::string& libBaseName,
                             ADFun<CGD>& fun,
                             const std::vector<std::set<size_t> >& relatedDepCandidates,
                             const std::vector<Base>& xTypical,
                             JacobianADMode jacMode,
                             bool jacobian = true,
                             bool hessian = true,
                             bool forReverseOne = false,
                             bool reverseTwo = false) {
        bool withLoops = !relatedDepCandidates.empty();

        assert(fun.Domain() == xTypical.size());

        /**
         * Create the source code
         */
        modelSourceGen_.reset(new ModelCSourceGen<double>(fun, libBaseName + (withLoops ? "Loops" : "NoLoops")));
        modelSourceGen_->setCreateForwardZero(true);
        modelSourceGen_->setJacobianADMode(jacMode);
        modelSourceGen_->setCreateSparseJacobian(jacobian);
        modelSourceGen_->setCreateSparseHessian(hessian);
        modelSourceGen_->setCreateForwardOne(forReverseOne && jacMode == JacobianADMode::Forward);
        modelSourceGen_->setCreateReverseOne(forReverseOne && jacMode == JacobianADMode::Reverse);
        modelSourceGen_->setCreateReverseTwo(reverseTwo);
        modelSourceGen_->setRelatedDependents(relatedDepCandidates);
        modelSourceGen_->setTypicalIndependentValues(xTypical);

        if (!customJacSparsity_.empty())
            modelSourceGen_->setCustomSparseJacobianElements(customJacSparsity_);

        if (!customHessSparsity_.empty())
            modelSourceGen_->setCustomSparseHessianElements(customHessSparsity_);

        libSourceGen_.reset(new ModelLibraryCSourceGen<double>(*modelSourceGen_.get()));
        libSourceGen_->setVerbose(this->verbose_);
        libSourceGen_->addListener(listener_);

        SaveFilesModelLibraryProcessor<double>::saveLibrarySourcesTo(*libSourceGen_.get(), "sources_" + libBaseName);

        if (!relatedDepCandidates.empty())
            patternDection_.push_back(listener_.patternDection);
        graphGen_.push_back(listener_.graphGen);
        srcCodeGen_.push_back(listener_.srcCodeGen);
    }

    inline void createDynamicLib(const std::string& libBaseName,
                                 const std::vector<Base>& xTypical,
                                 bool loadLib,
                                 bool withLoops) {
        /**
         * Create the dynamic library
         * (compile source code)
         */
        DynamicModelLibraryProcessor<double> p(*libSourceGen_.get(), std::string("modelLib") + (withLoops ? "Loops" : "NoLoops"));
        GccCompiler<double> compiler;
        if (!compileFlags_.empty())
            compiler.setCompileFlags(compileFlags_);
#ifndef NDEBUG
        compiler.setSourcesFolder("sources_" + libBaseName);
        compiler.setSaveToDiskFirst(true);
#endif
        dynamicLib_ = p.createDynamicLibrary(compiler, loadLib);
        if (loadLib) {
            model_ = dynamicLib_->model(libBaseName + (withLoops ? "Loops" : "NoLoops"));
            assert(model_.get() != nullptr);
            for (size_t i = 0; i < externalModels_.size(); i++)
                model_->addExternalModel(*externalModels_[i]);
        }

        srcCodeComp_.push_back(listener_.srcCodeComp);
        dynLibComp_.push_back(listener_.dynLibComp);
        total_.push_back(listener_.totalLibrary);
    }

    inline void createDynamicLib(ADFun<CGD>& fun,
                                 const std::vector<std::set<size_t> >& relatedDepCandidates,
                                 const std::vector<Base>& xTypical,
                                 bool loadLib,
                                 JacobianADMode jacMode,
                                 bool jacobian = true,
                                 bool hessian = true,
                                 bool forReverseOne = false,
                                 bool reverseTwo = false) {
        listener_.reset();

        std::string libBaseName = createLibBaseName(jacMode, forReverseOne, reverseTwo);
        createSource(libBaseName, fun, relatedDepCandidates, xTypical, jacMode, jacobian, hessian, forReverseOne, reverseTwo);

        bool withLoops = !relatedDepCandidates.empty();
        createDynamicLib(libBaseName, xTypical, loadLib, withLoops);
    }

    inline void createJitModelLib(const std::string& libBaseName,
                                  const std::vector<Base>& xTypical,
                                  bool withLoops) {
        listener_.reset();

        /**
         * Prepare JITed library
         */
        llvmLib_ = LlvmModelLibraryProcessor<Base>::create(*libSourceGen_);
        model_ = llvmLib_->model(libBaseName + (withLoops ? "Loops" : "NoLoops")); //must request model
        assert(model_.get() != nullptr);
        for (size_t i = 0; i < externalModels_.size(); i++)
            model_->addExternalModel(*externalModels_[i]);

        jit_.push_back(listener_.jit);
        total_.push_back(listener_.totalLibrary);
    }

    inline void executionSpeedCppADCG(const std::vector<double>& x,
                                      bool eval,
                                      bool zero = true,
                                      bool jacobian = true,
                                      bool hessian = true) {
        using namespace std::chrono;

        // model (zero-order)
        if (zero) {
            std::vector<duration> dt;
            if (eval && zeroOrder) {
                dt.resize(nTimes_);
                std::vector<double> y(model_->Range());
                for (size_t i = 0; i < nTimes_; i++) {
                    auto t0 = steady_clock::now();
                    model_->ForwardZero(x, y);
                    dt[i] = steady_clock::now() - t0;
                }
            }
            // save result
            printStat("zero order", dt);
        }

        // Jacobian
        if (jacobian) {
            std::vector<duration> dt;
            if (eval && sparseJacobian) {
                dt.resize(nTimes_);
                std::vector<double> jac;
                std::vector<size_t> rows, cols;

                for (size_t i = 0; i < nTimes_; i++) {
                    auto t0 = steady_clock::now();
                    model_->SparseJacobian(x, jac, rows, cols);
                    dt[i] = steady_clock::now() - t0;
                }
            }
            // save result
            printStat("jacobian", dt);
        }

        // Hessian
        if (hessian) {
            std::vector<duration> dt;
            if (eval && sparseHessian) {
                dt.resize(nTimes_);
                std::vector<double> w(model_->Range(), 1.0);
                std::vector<double> hess;
                std::vector<size_t> rows, cols;

                for (size_t i = 0; i < nTimes_; i++) {
                    auto t0 = steady_clock::now();
                    model_->SparseHessian(x, w, hess, rows, cols);
                    dt[i] = steady_clock::now() - t0;
                }
            }
            // save result
            printStat("hessian", dt);
        }

    }

    inline void executionSpeedCppAD(ADFun<Base>& fun,
                                    const std::vector<double>& x,
                                    bool zero = true,
                                    bool jacobian = true,
                                    bool hessian = true) {
        using namespace std::chrono;

        // model (zero-order)
        if (zero) {
            std::vector<duration> dt;
            if (cppAD && zeroOrder) {
                std::vector<double> y(fun.Range());
                dt.resize(nTimes_);
                for (size_t i = 0; i < nTimes_; i++) {
                    auto t0 = steady_clock::now();
                    y = fun.Forward(0, x);
                    dt[i] = steady_clock::now() - t0;
                }
            }
            // save result
            printStat("zero order", dt);
        }

        // Jacobian
        if (jacobian) {
            size_t nTimes = cppAD ? (preparation ? nTimes_ : sparseJacobian ? 1 : 0) : 0;
            std::vector<duration> dtp(nTimes);

            std::vector<std::set<size_t> > sparsity;
            for (size_t i = 0; i < dtp.size(); i++) {
                auto t0 = steady_clock::now();
                sparsity = CppAD::cg::jacobianForwardSparsitySet<std::vector<std::set<size_t> >, Base>(fun);
                dtp[i] = steady_clock::now() - t0;
            }
            printStat("jacobian sparsity", dtp);

            std::vector<duration> dt;
            if (cppAD && sparseJacobian) {
                std::vector<size_t> rows, cols;
                CppAD::cg::generateSparsityIndexes(sparsity, rows, cols);
                std::vector<double> jac(rows.size());

                sparse_jacobian_work work;
                dt.resize(nTimes_);
                for (size_t i = 0; i < nTimes_; i++) {
                    auto t0 = steady_clock::now();
                    fun.SparseJacobianReverse(x, sparsity, rows, cols, jac, work);
                    dt[i] = steady_clock::now() - t0;
                }
            }
            // save result
            printStat("jacobian", dt);
        }

        // Hessian
        if (hessian) {
            size_t nTimes = cppAD ? (preparation ? nTimes_ : sparseHessian ? 1 : 0) : 0;
            std::vector<duration> dtp(nTimes);

            std::vector<std::set<size_t> > sparsity;
            for (size_t i = 0; i < dtp.size(); i++) {
                auto t0 = steady_clock::now();
                sparsity = CppAD::cg::hessianSparsitySet<std::vector<std::set<size_t> >, Base>(fun);
                dtp[i] = steady_clock::now() - t0;
            }
            printStat("hessian sparsity", dtp);

            std::vector<duration> dt;
            if (cppAD && sparseHessian) {
                std::vector<size_t> rows, cols;
                CppAD::cg::generateSparsityIndexes(sparsity, rows, cols);
                std::vector<double> hess(rows.size());

                std::vector<double> w(model_->Range(), 1.0);

                sparse_hessian_work work;

                dt.resize(nTimes_);
                for (size_t i = 0; i < nTimes_; i++) {
                    auto t0 = steady_clock::now();
                    fun.SparseHessian(x, w, sparsity, rows, cols, hess, work);
                    dt[i] = steady_clock::now() - t0;
                }
            }
            // save result
            printStat("hessian", dt);

        }
    }

    inline static double mean(std::vector<double> v) {
        double avg = 0;
        for (size_t i = 0; i < v.size(); i++)
            avg += v[i];
        avg /= v.size();
        return avg;
    }

    inline static double stdDev(std::vector<double> v) {
        double avg = mean(v);
        double sum = 0;
        for (size_t i = 0; i < v.size(); i++)
            sum += (v[i] - avg) * (v[i] - avg);
        sum /= v.size();
        return sqrt(sum);
    }
};

} // END cg namespace
} // END CppAD namespace

#endif
