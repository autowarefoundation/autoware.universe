#ifndef CPPAD_CG_TEST_COLLOCATION_INCLUDED
#define CPPAD_CG_TEST_COLLOCATION_INCLUDED
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

#include "gccCompilerFlags.hpp"
#include "../patterns/pattern_test_model.hpp"

namespace CppAD {
namespace cg {

template<class T>
class CollocationModel : public PatternTestModel<T> {
protected:
    const size_t K_; // order of collocation

    const size_t ns_; // number of states of the atomic function
    const size_t nm_; // number of controls of the atomic function
    const size_t npar_; // number of parameters of the atomic function
    const size_t na_; // number of independent variables of the atomic function

    std::vector<double> xa_; // default atomic model values

    std::unique_ptr<DynamicLib<double> > atomicDynamicLib_;
    std::unique_ptr<GenericModel<double> > atomicModel_;
    std::vector<atomic_base<T>*> atoms_;
    bool ignoreParameters_;
    bool verbose_;
private:
    std::unique_ptr<CGAtomicFun<double> > atomModel_;
public:

    inline CollocationModel(size_t ns,
                            size_t nm,
                            size_t npar) :
        K_(3),
        ns_(ns),
        nm_(nm),
        npar_(npar),
        na_(ns + nm + npar),
        ignoreParameters_(true),
        verbose_(false) {
    }

    inline size_t getAtomicIndepCount() const {
        return na_;
    }

    inline size_t getAtomicDepCount() const {
        return ns_;
    }

    inline void setTypicalAtomModelValues(const std::vector<double>& xxx) {
        assert(xxx.size() == na_);
        xa_ = xxx;
    }

    inline void setIgnoreParameters(bool ignoreParameters) {
        ignoreParameters_ = ignoreParameters;
    }

    inline std::vector<double> getTypicalAtomicValues() {
        return xa_;
    }

    inline std::vector<double> getTypicalValues(size_t repeat) {
        assert(xa_.size() == na_);

        /**
         * collocation model values
         */
        size_t nvarsk = ns_;
        size_t nMstart = npar_ + nvarsk * K_ * repeat + nvarsk;

        std::vector<double> x(nMstart + repeat * nm_, 1.0);
        // parameters
        for (size_t j = 0; j < npar_; j++)
            x[j] = xa_[ns_ + nm_ + j];

        size_t s = npar_;

        // i = 0 K = 0
        // states
        for (size_t j = 0; j < ns_; j++) {
            x[s++] = xa_[j];
        }

        for (size_t i = 0; i < repeat; i++) {
            // controls
            for (size_t j = 0; j < nm_; j++) {
                x[nMstart + nm_ * i + j] = xa_[ns_ + j];
            }

            // K = 1
            // states
            for (size_t j = 0; j < ns_; j++) {
                x[s] = 1.0 + 0.01 * i;
                x[s++] = xa_[j];
            }

            // K = 2
            // states
            for (size_t j = 0; j < ns_; j++) {
                x[s++] = xa_[j];
            }

            // K = 3
            // states
            for (size_t j = 0; j < ns_; j++) {
                x[s++] = xa_[j];
            }
        }

        return x;
    }

    inline size_t varCount(size_t repeat) {
        return npar_ + repeat * (nm_ + K_ * ns_);
    }

    std::vector<AD<T> > evaluateModel(const std::vector<AD<T> >& x,
                                      size_t repeat) {
        atomic_base<T>& atomModel = createAtomic();
        return evaluateModel<T>(x, repeat, atomModel);
    }

    template<class T2>
    std::vector<AD<T2> > evaluateModel(const std::vector<AD<T2> >& x,
                                       size_t repeat,
                                       atomic_base<T2>& atomModel) {

        size_t m2 = repeat * K_ * ns_;

        // dependent variable vector
        std::vector<AD<T2> > dep(m2);

        std::vector<AD<T2> > dxikdt(ns_);
        std::vector<AD<T2> > xik(na_);

        // parameters
        for (size_t j = 0; j < npar_; j++)
            xik[ns_ + nm_ + j] = x[j];

        size_t s = npar_;
        size_t nvarsk = ns_;
        size_t nMstart = npar_ + nvarsk * K_ * repeat + nvarsk;
        size_t eq = 0;

        for (size_t i = 0; i < repeat; i++) {
            size_t s0 = s;

            // controls
            for (size_t j = 0; j < nm_; j++) {
                xik[ns_ + j] = x[nMstart + nm_ * i + j];
            }

            // K = 1
            for (size_t j = 0; j < ns_; j++) {
                xik[j] = x[s + j]; // states
            }
            s += nvarsk;
            // xik[ns + nm + npar] = x[s + ns];// time

            atomModel(xik, dxikdt); // ODE
            for (size_t j = 0; j < ns_; j++) {
                dep[eq + j] = dxikdt[j]
                        + 0.13797958971132715 * x[s0 + j]
                        + -0.10749149571305303 * x[s0 + nvarsk + j]
                        + -0.038928002823013501 * x[s0 + 2 * nvarsk + j]
                        + 0.008439908824739363 * x[s0 + 3 * nvarsk + j];
            }
            eq += ns_;

            // K = 2
            for (size_t j = 0; j < ns_; j++) {
                xik[j] = x[s + j]; // states
            }
            s += nvarsk;
            // xik[ns + nm + npar] = x[s + ns];// time

            atomModel(xik, dxikdt); // ODE
            for (size_t j = 0; j < ns_; j++) {
                dep[eq + j] = dxikdt[j]
                        + -0.057979589711327127 * x[s0 + j]
                        + 0.11892800282301351 * x[s0 + nvarsk + j]
                        + -0.025841837620280327 * x[s0 + 2 * nvarsk + j]
                        + -0.035106575491406049 * x[s0 + 3 * nvarsk + j];
            }
            eq += ns_;

            // K = 3
            for (size_t j = 0; j < ns_; j++) {
                xik[j] = x[s + j]; // states
            }
            s += nvarsk;
            // xik[ns + nm + npar] = x[s + ns];// time

            atomModel(xik, dxikdt); // ODE
            for (size_t j = 0; j < ns_; j++) {
                dep[eq + j] = dxikdt[j]
                        + 0.099999999999999978 * x[s0 + j]
                        + -0.18439908824739357 * x[s0 + nvarsk + j]
                        + 0.25106575491406025 * x[s0 + 2 * nvarsk + j]
                        + -0.16666666666666669 * x[s0 + 3 * nvarsk + j];
            }
            eq += ns_;

        }

        return dep;
    }

    void createAtomicLib() {
        using CGD = CG<double>;
        using ADCGD = AD<CGD>;

        /**
         * Tape model
         */
        std::vector<ADCGD> xa(na_);
        for (size_t j = 0; j < na_; j++)
            xa[j] = xa_[j];
        CppAD::Independent(xa);

        std::vector<ADCGD> ya(ns_);

        atomicFunction(xa, ya);

        ADFun<CGD> fun;
        fun.Dependent(ya);

        /**
         * Compile
         */
        std::string lName = getAtomicLibName()+(ignoreParameters_ ? "" : "All");
        ModelCSourceGen<double> compHelpL(fun, lName);
        compHelpL.setCreateForwardZero(true);
        compHelpL.setCreateForwardOne(true);
        compHelpL.setCreateReverseOne(true);
        compHelpL.setCreateReverseTwo(true);
        compHelpL.setTypicalIndependentValues(xa_);

        if (ignoreParameters_) {
            std::vector<std::set<size_t> > jacSparAll = jacobianSparsitySet<std::vector<std::set<size_t> > >(fun);
            std::vector<std::set<size_t> > jacSpar(jacSparAll.size());
            for (size_t i = 0; i < jacSparAll.size(); i++) {
                // only differential information for states and controls
                std::set<size_t>::const_iterator itEnd = jacSparAll[i].upper_bound(ns_ + nm_ - 1);
                if (itEnd != jacSparAll[i].begin())
                    jacSpar[i].insert(jacSparAll[i].begin(), itEnd);
            }
            compHelpL.setCustomSparseJacobianElements(jacSpar);

            std::vector<std::set<size_t> > hessSparAll = hessianSparsitySet<std::vector<std::set<size_t> > >(fun);
            std::vector<std::set<size_t> > hessSpar(hessSparAll.size());
            for (size_t i = 0; i < ns_ + nm_; i++) {
                std::set<size_t>::const_iterator it = hessSparAll[i].upper_bound(i); // only the lower left side
                if (it != hessSparAll[i].begin())
                    hessSpar[i].insert(hessSparAll[i].begin(), it);
            }
            compHelpL.setCustomSparseHessianElements(hessSpar);
        }

        ModelLibraryCSourceGen<double> compDynHelpL(compHelpL);
        compDynHelpL.setVerbose(verbose_);

        //SaveFilesModelLibraryProcessor<double>::saveLibrarySourcesTo(compDynHelpL, "sources_" + lName);

        DynamicModelLibraryProcessor<double> p(compDynHelpL, lName);
        GccCompiler<double> compiler;
        prepareTestCompilerFlags(compiler);
        compiler.setSourcesFolder("sources_" + lName);
        compiler.setSaveToDiskFirst(true);

        atomicDynamicLib_ = p.createDynamicLibrary(compiler);

        /**
         * load the model
         */
        atomicModel_ = atomicDynamicLib_->model(lName);
    }

    GenericModel<double>* getGenericModel() {
        return atomicModel_.get();
    }

    CGAtomicGenericModel<double>& getDoubleAtomic() {
        return atomicModel_->asAtomic();
    }

protected:
    virtual void atomicFunction(const std::vector<AD<CG<double> > >& x,
                                std::vector<AD<CG<double> > >& y) = 0;

    virtual void atomicFunction(const std::vector<AD<double> >& x,
                                std::vector<AD<double> >& y) = 0;

    virtual std::string getAtomicLibName() = 0;

    atomic_base<T>& createAtomic();
};

template<>
atomic_base<CG<double> >& CollocationModel<CG<double> >::createAtomic() {
    size_t n = atomicModel_->Domain();
    atomModel_.reset(new CGAtomicFun<double>(atomicModel_->asAtomic(), std::vector<double>(n), true));
    return *atomModel_.get();
}

template<>
atomic_base<double>& CollocationModel<double>::createAtomic() {
    return atomicModel_->asAtomic();
}

} // END cg namespace
} // END CppAD namespace

#endif
