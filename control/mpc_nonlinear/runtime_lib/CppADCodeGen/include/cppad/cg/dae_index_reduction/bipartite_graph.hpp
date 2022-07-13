#ifndef CPPAD_CG_BIPARTITE_GRAPH_INCLUDED
#define CPPAD_CG_BIPARTITE_GRAPH_INCLUDED
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

#include <cppad/cg/dae_index_reduction/bipartite_nodes.hpp>
#include <cppad/cg/dae_index_reduction/dae_equation_info.hpp>
#include <cppad/cg/dae_index_reduction/time_diff.hpp>

namespace CppAD {
namespace cg {

/**
 * Bipartite graph which holds nodes to represent variables and equations
 * in a DAE system.
 */
template<class Base>
class BipartiteGraph {
protected:
    using CGBase = CppAD::cg::CG<Base>;
    using ADCG = CppAD::AD<CGBase>;
protected:
    /**
     * The original model
     */
    ADFun<CG<Base> >* const fun_;
    /**
     * DAE variable information for the original system
     */
    std::vector<DaeVarInfo> varInfo_;
    /**
     * original sparsity pattern
     */
    std::vector<bool> sparsity_;
    // Bipartite graph ([equation i][variable j])
    std::vector<Vnode<Base>*> vnodes_;
    std::vector<Enode<Base>*> enodes_;
    /**
     * the maximum order of the time derivatives in the original model
     */
    int origMaxTimeDivOrder_;
    /**
     * the number of time dependent variables in the original model
     */
    size_t origTimeDependentCount_;
    /**
     * Defines whether or not original names saved by using
     * CppAD::PrintFor(0, "", val, name)
     * should be kept by also adding PrintFor operations in the reduced model.
     */
    bool preserveNames_;
private:
    int timeOrigVarIndex_; // time index in the original user model (may not exist)
    SimpleLogger& logger_;
public:

    /**
     * Creates the bipartite graph.
     *
     * @param fun The DAE model
     * @param varInfo DAE model variable classification
     * @param eqName Equation names (it can be an empty vector)
     */
    BipartiteGraph(ADFun<CG<Base> >& fun,
                   const std::vector<DaeVarInfo>& varInfo,
                   const std::vector<std::string>& eqName,
                   SimpleLogger& logger) :
            fun_(&fun),
            varInfo_(varInfo),
            origMaxTimeDivOrder_(0),
            origTimeDependentCount_(0),
            preserveNames_(false),
            timeOrigVarIndex_(-1),
            logger_(logger) {

        using namespace std;
        using std::vector;

        CPPADCG_ASSERT_UNKNOWN(fun_ != nullptr);
        const size_t m = fun.Range(); // equation count
        const size_t n = fun.Domain(); // total variable count

        CPPADCG_ASSERT_UNKNOWN(varInfo_.size() == n);
        for (size_t j = 0; j < n; ++j) {
            varInfo_[j].setOriginalIndex(j);
            varInfo_[j].setId(j);
        }

        for (size_t j = 0; j < n; ++j) {
            int deriv = varInfo_[j].getAntiDerivative();
            CPPADCG_ASSERT_UNKNOWN(deriv < int(varInfo_.size()));
            if (deriv >= 0) {
                varInfo_[deriv].setDerivative(j);
            }
        }

        for (size_t j = 0; j < n; ++j) {
            determineVariableOrder(varInfo_[j]);
        }

        // create equation nodes
        enodes_.reserve(1.2 * m + 1);
        enodes_.resize(m);
        for (size_t i = 0; i < m; i++) {
            if (i < eqName.size())
                enodes_[i] = new Enode<Base>(i, eqName[i]);
            else
                enodes_[i] = new Enode<Base>(i);
        }

        // locate the time variable (if present)
        for (size_t dj = 0; dj < n; dj++) {
            if (varInfo_[dj].isIntegratedVariable()) {
                if (timeOrigVarIndex_ >= 0) {
                    throw CGException("More than one time variable (integrated variable) defined");
                }
                timeOrigVarIndex_ = dj;
            }
        }

        // determine the order of each time derivative
        vector<int> derivOrder = determineVariableDiffOrder(varInfo_);
        map<int, vector<size_t> > order2Tape;
        for (size_t tape = 0; tape < derivOrder.size(); ++tape) {
            order2Tape[derivOrder[tape]].push_back(tape);
        }
        origMaxTimeDivOrder_ = *std::max_element(derivOrder.begin(), derivOrder.end());

        /**
         * generate names for the variables
         */
        std::string timeVarName;
        if (timeOrigVarIndex_ < 0) {
            timeVarName = "t";
        } else {
            if (varInfo_[timeOrigVarIndex_].getName().empty()) {
                varInfo_[timeOrigVarIndex_].setName("t");
            }
            timeVarName = varInfo_[timeOrigVarIndex_].getName();
        }

        stringstream ss;
        for (int order = -1; order <= origMaxTimeDivOrder_; order++) {
            //size_t j = 0; j < varInfo_.size(); j++
            const vector<size_t>& tapeIndexes = order2Tape[order];
            if (order < 0) {
                for (size_t p = 0; p < tapeIndexes.size(); ++p) {
                    DaeVarInfo& var = varInfo_[tapeIndexes[p]];
                    if (var.getName().empty()) {
                        ss << "p" << p;
                        var.setName(ss.str());
                        ss.str("");
                        ss.clear();
                    }
                }

            } else if (order == 0) {
                for (size_t p = 0; p < tapeIndexes.size(); ++p) {
                    DaeVarInfo& var = varInfo_[tapeIndexes[p]];
                    if (var.getName().empty()) {
                        ss << "x" << p;
                        var.setName(ss.str());
                        ss.str("");
                        ss.clear();
                    }
                }
            } else if (order > 0) {
                for (size_t p = 0; p < tapeIndexes.size(); ++p) {
                    DaeVarInfo& var = varInfo_[tapeIndexes[p]];
                    if (var.getName().empty()) {
                        const DaeVarInfo& deriv = varInfo_[var.getAntiDerivative()];
                        var.setName("d" + deriv.getName() + "d" + timeVarName);
                    }
                }
            }
        }

        // sort the variables according to the time derivative order (constants are kept out)
        vector<size_t> new2Tape;
        vector<int> tape2New(n, -1);
        new2Tape.reserve(n);
        for (int order = 0; order <= origMaxTimeDivOrder_; order++) {
            const vector<size_t>& tapeIndexes = order2Tape[order];
            for (size_t p = 0; p < tapeIndexes.size(); ++p) {
                size_t tapeIndex = tapeIndexes[p];
                tape2New[tapeIndex] = new2Tape.size();
                new2Tape.push_back(tapeIndex);
            }
        }

        // create the variable nodes
        origTimeDependentCount_ = new2Tape.size();
        vnodes_.resize(origTimeDependentCount_);
        for (size_t j = 0; j < vnodes_.size(); j++) {
            size_t tapeIndex = new2Tape[j];
            int tapeIndex0 = varInfo_[tapeIndex].getAntiDerivative();
            const std::string& name = varInfo_[tapeIndex].getName();

            CPPADCG_ASSERT_UNKNOWN(varInfo_[tapeIndex].isFunctionOfIntegrated());

            if (tapeIndex0 < 0) {
                // generate the variable name
                vnodes_[j] = new Vnode<Base>(j, tapeIndex, name);
            } else {
                Vnode<Base>* derivativeOf = vnodes_[tape2New[tapeIndex0]];
                vnodes_[j] = new Vnode<Base>(j, tapeIndex, derivativeOf, name);
            }
        }

        // create the edges
        sparsity_ = jacobianSparsity<vector<bool>, CGBase>(fun);

        for (size_t i = 0; i < m; i++) {
            for (size_t p = 0; p < n; p++) {
                int j = tape2New[p];
                if (j >= 0 && sparsity_[i * n + p]) {
                    enodes_[i]->addVariable(vnodes_[j]);
                }
            }
        }

        // make sure the system is not under or over determined
        size_t nvar = 0;
        for (size_t j = 0; j < vnodes_.size(); j++) {
            const Vnode<Base>* jj = vnodes_[j];
            if (!jj->isParameter() && // exclude constants
                (jj->antiDerivative() != nullptr || // derivatives
                 jj->derivative() == nullptr) // algebraic variables
                    ) {
                nvar++;
            }
        }

        if (nvar != m) {
            throw CGException("The system is not well determined. "
                                      "The of number of equations (", enodes_.size(), ")"
                                      " does not match the number of unknown variables "
                                      "(", nvar, ").");
        }
    }

    BipartiteGraph(const BipartiteGraph& p) = delete;

    BipartiteGraph& operator=(const BipartiteGraph& p) = delete;

    virtual ~BipartiteGraph() {
        for (size_t i = 0; i < enodes_.size(); i++)
            delete enodes_[i];

        for (size_t j = 0; j < vnodes_.size(); j++)
            delete vnodes_[j];
    }


    inline std::vector<Vnode<Base>*>& variables() {
        return vnodes_;
    }

    inline const std::vector<Vnode<Base>*>& variables() const {
        return vnodes_;
    }

    inline std::vector<Enode<Base>*>& equations() {
        return enodes_;
    }

    inline const std::vector<Enode<Base>*>& equations() const {
        return enodes_;
    }

    const std::vector<DaeVarInfo>& getOriginalVariableInfo() const {
        return varInfo_;
    }

    inline size_t getOrigTimeDependentCount() const {
        return origTimeDependentCount_;
    }

    /**
     * Defines whether or not original names saved by using
     * CppAD::PrintFor(0, "", val, name)
     * should be kept by also adding PrintFor operations in the reduced model.
     */
    void setPreserveNames(bool p) {
        preserveNames_ = p;
    }

    /**
     * Whether or not original names saved by using
     * CppAD::PrintFor(0, "", val, name)
     * should be kept by also adding PrintFor operations in the reduced model.
     */
    bool isPreserveNames() const {
        return preserveNames_;
    }

    /**
     * Provides the structural index after this graph has been reduced.
     *
     * @return the DAE differentiation index.
     */
    inline size_t getStructuralIndex() const {
        size_t origM = this->fun_->Range();
        if (origM == enodes_.size()) {
            // no index reduction performed: it is either an index 1 DAE or an ODE
            bool isDAE = false;
            for (size_t j = 0; j < varInfo_.size(); j++) {
                const DaeVarInfo& jj = varInfo_[j];
                if (jj.getDerivative() < 0 && !jj.isIntegratedVariable() && jj.isFunctionOfIntegrated()) {
                    isDAE = true; // found algebraic variable
                    break;
                }
            }
            if (!isDAE) {
                return 0;
            } else {
                return 1;
            }
        }

        size_t index = 0;
        for (size_t i = origM; i < enodes_.size(); i++) {
            Enode<Base>* ii = enodes_[i];
            size_t eqOrder = 0;
            if (ii->derivative() == nullptr) {
                Enode<Base>* eq = ii;
                while (eq->derivativeOf() != nullptr) {
                    eq = eq->derivativeOf();
                    eqOrder++;
                }
                if (eqOrder > index)
                    index = eqOrder;
            }
        }

        return index + 1; // one extra differentiation to get an ODE
    }

    inline void printResultInfo(const std::string& method) {
        logger_.log() << "\n" << method << " DAE differentiation/structural index reduction:\n\n"
                "   Equations count: " << enodes_.size() << "\n";
        for (Enode<Base>* ii : enodes_) {
            logger_.log() << "      " << ii->index() << " - " << *ii << "\n";
        }

        logger_.log() << "\n   Variable count: " << vnodes_.size() << "\n";

        for (const Vnode<Base>* jj : vnodes_) {
            logger_.log() << "      " << jj->index() << " - " << *jj;
            if (jj->assignmentEquation() != nullptr) {
                logger_.log() << " assigned to " << *jj->assignmentEquation() << "\n";
            } else if (jj->isParameter()) {
                logger_.log() << " is a parameter (time independent)\n";
            } else {
                logger_.log() << " NOT assigned to any equation\n";
            }
        }

        logger_.log() << "\n   Degrees of freedom: " << vnodes_.size() - enodes_.size() << std::endl;
    }

    inline void uncolorAll() {
        for (Vnode<Base>* j : vnodes_) {
            j->uncolor();
        }

        for (Enode<Base>* i : enodes_) {
            i->uncolor();
        }
    }

    inline Vnode<Base>* createDerivate(Vnode<Base>& j) {
        if (j.derivative() != nullptr)
            return j.derivative();

        // add new variable derivatives of colored variables
        size_t newVarCount = vnodes_.size() - origTimeDependentCount_;
        size_t tapeIndex = varInfo_.size() + newVarCount;

        Vnode<Base>* jDiff = new Vnode<Base> (vnodes_.size(), tapeIndex, &j);
        vnodes_.push_back(jDiff);

        if (logger_.getVerbosity() >= Verbosity::High)
            logger_.log() << "Created " << *jDiff << "\n";

        return jDiff;
    }

    inline Enode<Base>* createDerivate(Enode<Base>& i,
                                       bool addOrigVars = true) {
        if (i.derivative() != nullptr)
            return i.derivative();

        Enode<Base>* iDiff = new Enode<Base> (enodes_.size(), &i);
        enodes_.push_back(iDiff);

        // differentiate newI and create edges!!!
        dirtyDifferentiateEq(i, *iDiff, addOrigVars);

        if (logger_.getVerbosity() >= Verbosity::High)
            logger_.log() << "Created " << *iDiff << "\n";

        return iDiff;
    }

    /**
     * Completely removes an equation and any variable that is only referenced
     * by this equation from the graph.
     * The equation cannot have a differentiated version of itself in the graph.
     *
     * @warning This equation node cannot be referenced after this call and
     * neither can the variables which are also removed for not being present
     * in any equation.
     */
    inline void remove(const Enode<Base>& i) {
        CPPADCG_ASSERT_UNKNOWN(enodes_[i.index()] == &i);
        CPPADCG_ASSERT_UNKNOWN(i.derivative() == nullptr);

        for (Vnode<Base>* j: i.variables()) {
            // remove the edges (connections in variables)
            auto& eqs = j->equations();
            auto it = std::find(eqs.begin(), eqs.end(), &i);
            CPPADCG_ASSERT_UNKNOWN(it != eqs.end());
            eqs.erase(it);

            /**
             * remove variable
             */
            while(j->equations().empty()) {
                CPPADCG_ASSERT_UNKNOWN(vnodes_[j->index()] == j);

                if (j->derivative() == nullptr) {
                    vnodes_.erase(vnodes_.cbegin() + j->index());

                    // update variable indices
                    for (size_t jj = j->index(); jj < vnodes_.size(); ++jj) {
                        vnodes_[jj]->setTapeIndex(vnodes_[jj]->tapeIndex() - 1);
                        vnodes_[jj]->setIndex(vnodes_[jj]->index() - 1);
                    }

                    auto* jOrig = j->antiDerivative();
                    CPPADCG_ASSERT_UNKNOWN(jOrig != nullptr);
                    jOrig->setDerivative(nullptr);

                    delete j; // no longer required
                    j = jOrig;
                }
            }

        }

        // update equation indices
        for (size_t ii = i.index() + 1; ii < enodes_.size(); ++ii) {
            CPPADCG_ASSERT_UNKNOWN(enodes_[ii]->index() > 0);
            CPPADCG_ASSERT_UNKNOWN(enodes_[ii]->index() == ii);
            enodes_[ii]->setIndex(enodes_[ii]->index() - 1);
        }

        if(i.derivativeOf() != nullptr) {
            i.derivativeOf()->setDerivative(nullptr);
        }

        auto it = std::find(enodes_.begin(), enodes_.end(), &i);
        CPPADCG_ASSERT_UNKNOWN(it != enodes_.end());
        enodes_.erase(it);

        delete &i; // no longer required
    }

    /**
     * Adds edges to a new equation resulting from the differentiation
     * of another assuming the new equation differential contains
     * all variables present in the original equation and their time
     * derivatives (not exactly correct but it works because the
     * potentially extra variables are removed later in the Pantelides method).
     *
     * An example with incorrectly added variables would be the dirty
     * differentiation of:
     *   x1 + x2 == 0
     * wich include the variables [x1, dx1dt, x2, dx2dt] although it should
     * only be [dx1dt, dx2dt].
     *
     * @param i equation node to differentiate
     * @throws CGException
     */
    inline void dirtyDifferentiateEq(Enode<Base>& i,
                                     Enode<Base>& iDiff,
                                     bool addOrigVars = true) {
        for (Vnode<Base>* jj : i.originalVariables()) {
            if(addOrigVars) {
                iDiff.addVariable(jj);
            }

            if (jj->derivative() != nullptr) {
                iDiff.addVariable(jj->derivative());
            } else if(!jj->isParameter()) {
                iDiff.addVariable(createDerivate(*jj));
            }
        }
    }

    /**
     * Creates a new tape for the index 1 model
     */
    inline std::unique_ptr<ADFun<CGBase>> generateNewModel(std::vector<DaeVarInfo>& newVarInfo,
                                                           std::vector<DaeEquationInfo>& equationInfo,
                                                           const std::vector<Base>& x) {
        using std::vector;

        std::unique_ptr<ADFun<CGBase> > reducedFun;

        vector<vector<Enode<Base>*> > newEquations;

        // find new equations that must be generated by differentiation
        vector<Enode<Base>*> newEqs;
        size_t origM = this->fun_->Range();
        for (size_t i = 0; i < origM; i++) {
            if (enodes_[i]->derivative() != nullptr) {
                CPPADCG_ASSERT_UNKNOWN(enodes_[i]->derivativeOf() == nullptr);
                newEqs.push_back(enodes_[i]->derivative());
            }
        }

        while (newEqs.size() > 0) {
            newEquations.push_back(newEqs);
            newEqs.clear();
            vector<Enode<Base>*>& eqs = newEquations.back();
            for (size_t i = 0; i < eqs.size(); i++) {
                if (eqs[i]->derivative() != nullptr) {
                    newEqs.push_back(eqs[i]->derivative());
                }
            }
        }

        if (newEquations.empty()) {
            // nothing to do
            return nullptr;
        }

        /**
         * Add the relationship between variables and derivatives
         */

        /**
         * Prepare the output information
         */
        newVarInfo = varInfo_; // copy
        size_t newVars = vnodes_.size() - origTimeDependentCount_;
        newVarInfo.reserve(varInfo_.size() + newVars);
        for (size_t j = origTimeDependentCount_; j < vnodes_.size(); j++) {
            // new variable derivative added by the Pantelides method
            Vnode<Base>* jj = vnodes_[j];
            CPPADCG_ASSERT_UNKNOWN(jj->antiDerivative() != nullptr);
            size_t antiDeriv = jj->antiDerivative()->tapeIndex();
            size_t id = newVarInfo.size();
            newVarInfo.push_back(DaeVarInfo(antiDeriv, jj->name(), id)); // create the new variable
            DaeVarInfo& newVar = newVarInfo.back();
            DaeVarInfo& newAntiDeriv = newVarInfo[antiDeriv];

            newAntiDeriv.setDerivative(jj->tapeIndex()); // update the antiderivative
            newVar.setOrder(newAntiDeriv.getOrder() + 1);
            newVar.setOriginalAntiDerivative(newVar.getOrder() == 1 ? newAntiDeriv.getOriginalIndex() : newAntiDeriv.getOriginalAntiDerivative());
            if (jj->derivative() != nullptr) {
                newVar.setDerivative(jj->derivative()->tapeIndex());
            }
        }

        std::map<Enode<Base>*, Vnode<Base>*> assignments;
        for (size_t j = 0; j < vnodes_.size(); j++) {
            Vnode<Base>* jj = vnodes_[j];
            if (jj->assignmentEquation() != nullptr) {
                assignments[jj->assignmentEquation()] = jj;
            }
        }

        equationInfo.resize(enodes_.size());
        for (size_t i = 0; i < enodes_.size(); i++) {
            Enode<Base>* ii = enodes_[i];
            int derivativeOf = ii->derivativeOf() != nullptr ? ii->derivativeOf()->index() : -1;
            int origIndex = ii->derivativeOf() == nullptr ? i : -1;
            int assignedVarIndex = assignments.count(ii) > 0 ? assignments[ii]->tapeIndex() : -1;

            equationInfo[i] = DaeEquationInfo(i, origIndex, derivativeOf, assignedVarIndex);
        }

        size_t timeTapeIndex;
        {
            CodeHandler<Base> handler;

            vector<CGBase> indep0(this->fun_->Domain());
            handler.makeVariables(indep0);

            const vector<CGBase> dep0 = forward0(*this->fun_, indep0);

            /**
             * generate a new tape
             */

            vector<ADCG> indepNew;
            if (timeOrigVarIndex_ >= 0) {
                indepNew = vector<ADCG>(newVarInfo.size()); // variables + time (vnodes include time)
                timeTapeIndex = timeOrigVarIndex_;
            } else {
                indepNew = vector<ADCG>(newVarInfo.size() + 1); // variables + time (new time variable added)
                timeTapeIndex = indepNew.size() - 1;
            }

            // initialize with the user provided values
            for (size_t j = 0; j < x.size(); j++) {
                indepNew[j] = x[j];
            }
            Independent(indepNew);

            // variables with the relationship between x dxdt and t
            vector<ADCG> indep2 = prepareTimeDependentVariables(indepNew, newVarInfo, timeTapeIndex);
            indep2.resize(indep0.size());

            Evaluator<Base, CGBase> evaluator0(handler);
            evaluator0.setPrintFor(preserveNames_); // variable names saved with CppAD::PrintFor
            vector<ADCG> depNew = evaluator0.evaluate(indep2, dep0);
            depNew.resize(enodes_.size());

            try {
                reducedFun.reset(new ADFun<CGBase>(indepNew, depNew));
            } catch (const std::exception& ex) {
                throw CGException("Failed to create ADFun: ", ex.what());
            }

            if (logger_.getVerbosity() >= Verbosity::High) {
                logger_.log() << "Original model:\n";
                printModel(logger_.log(), *reducedFun, newVarInfo, equationInfo);
            }
        }


        /**
         * generate the system of equations by repeatedly differentiating
         * and adding equations to the DAE system
         */
        for (size_t d = 0; d < newEquations.size(); d++) {
            vector<Enode<Base>*>& equations = newEquations[d];

            size_t m = reducedFun->Domain(); // total variable count
            //size_t n = reducedFun->Range(); // equation count

            /**
             * register operations from the other equations
             */
            CodeHandler<Base> handler0;

            vector<CGBase> indep0(m);
            handler0.makeVariables(indep0);

            vector<CGBase> dep = forward0(*reducedFun, indep0);

            /**
             * register operations used to differentiate the equations
             */
            //forwardTimeDiff(equations, dep, timeTapeIndex);
            reverseTimeDiff(*reducedFun, equations, dep, timeTapeIndex);

            /**
             * reconstruct the new system of equations
             */
            vector<ADCG> indep2;
            vector<ADCG> indepNew;

            if (d < newEquations.size() - 1) {
                indepNew.resize(m);
            } else if (timeOrigVarIndex_ < 0) {
                // the very last model creation
                indepNew.resize(m - 1); // take out time (it was added by this function and not the user)
            } else {
                // the very last model creation
                indepNew.resize(m);
            }

            for (size_t j = 0; j < x.size(); j++) {
                indepNew[j] = x[j];
            }
            Independent(indepNew);

            if (d < newEquations.size() - 1) {
                // variables with the relationship between x, dxdt and t
                indep2 = prepareTimeDependentVariables(indepNew, newVarInfo, timeTapeIndex);
            } else {
                indep2 = indepNew;
                indep2.resize(m);
            }

            Evaluator<Base, CGBase> evaluator(handler0);
            evaluator.setPrintFor(preserveNames_); // variable names saved with CppAD::PrintFor
            vector<ADCG> depNew = evaluator.evaluate(indep2, dep);

            try {
                reducedFun.reset(new ADFun<CGBase>(indepNew, depNew));
            } catch (const std::exception& ex) {
                throw CGException("Failed to create ADFun: ", ex.what());
            }

            if (logger_.getVerbosity() >= Verbosity::High) {
                logger_.log() << equations.size() << " new equations:\n";
                printModel(logger_.log(), *reducedFun, newVarInfo, equationInfo);
            }
        }

        return reducedFun;
    }

    inline static void forwardTimeDiff(ADFun<CGBase>& reducedFun,
                                       const std::vector<Enode<Base>*>& equations,
                                       std::vector<CG<Base> >& dep,
                                       size_t tapeTimeIndex) {

        size_t m = reducedFun.Domain();

        std::vector<CGBase> u(m, CGBase(0));
        u[tapeTimeIndex] = CGBase(1);
        std::vector<CGBase> v;
        try {
            v = reducedFun.Forward(1, u);
        } catch (const std::exception& ex) {
            throw CGException("Failed to determine model Jacobian (forward mode): ", ex.what());
        }

        for (size_t e = 0; e < equations.size(); e++) {
            dep[equations[e]->index()] = v[equations[e]->derivativeOf()->index()];
        }
    }

    inline static void reverseTimeDiff(ADFun<CGBase>& reducedFun,
                                       const std::vector<Enode<Base>*>& equations,
                                       std::vector<CG<Base> >& dep,
                                       size_t tapeTimeIndex) {
        size_t m = reducedFun.Domain();
        size_t n = reducedFun.Range();
        std::vector<CGBase> u(m);
        std::vector<CGBase> v(n);

        for (size_t e = 0; e < equations.size(); e++) {
            size_t i = equations[e]->derivativeOf()->index();
            if (reducedFun.Parameter(i)) { // return zero for this component of f
                dep[equations[e]->index()] = 0;
            } else {
                // set v to the i-th coordinate direction
                v[i] = 1;

                // compute the derivative of this component of f
                try {
                    u = reducedFun.Reverse(1, v);
                } catch (const std::exception& ex) {
                    throw CGException("Failed to determine model Jacobian (reverse mode): ", ex.what());
                }

                // reset v to vector of all zeros
                v[i] = 0;

                // return the result
                dep[equations[e]->index()] = u[tapeTimeIndex];
            }
        }
    }

    /**
     * Introduces a dependency with respect to time in the provided variables.
     *
     * @param indepOrig  The variables without time dependency
     *                    (in the original variable order).
     * @return The new variables with the time dependency
     *          (in the original variable order).
     */
    inline std::vector<CppAD::AD<CG<Base> > > prepareTimeDependentVariables(const std::vector<ADCG>& indepOrig,
                                                                            const std::vector<DaeVarInfo>& newVarInfo,
                                                                            size_t timeTapeIndex) const {
        CPPADCG_ASSERT_UNKNOWN(timeTapeIndex < indepOrig.size());

        using std::vector;
        using ADCGBase = CppAD::AD<CGBase>;

        vector<ADCGBase> indepOut(indepOrig.size());
        vector<ADCGBase> ax(3); // function inputs
        vector<ADCGBase> ay(1); // function output

        ax[2] = indepOrig[timeTapeIndex]; // time

        for (size_t j = 0; j < newVarInfo.size(); j++) {
            const DaeVarInfo& jj = newVarInfo[j];
            if (jj.getDerivative() >= 0) {
                ax[0] = indepOrig[j]; // x
                ax[1] = indepOrig[jj.getDerivative()]; // dxdt
                time_var(0, ax, ay);
                indepOut[j] = ay[0];
            } else {
                indepOut[j] = indepOrig[j];
            }
        }

        if (newVarInfo.size() < indepOrig.size()) {
            indepOut[indepOut.size() - 1] = indepOrig[timeTapeIndex];
        }

        return indepOut;
    }

    inline void printModel(std::ostream& out,
                           ADFun<CG<Base> >* fun) {
        printModel(out, fun, varInfo_);
    }

    /**
     * Prints out a DAE model to the standard output.
     *
     * @param fun  The taped model
     */
    inline void printModel(std::ostream& out,
                           ADFun<CG<Base> >& fun,
                           const std::vector<DaeVarInfo>& varInfo,
                           const std::vector<DaeEquationInfo>& eqInfo) const {
        std::vector<std::string> vnames(varInfo.size());
        for (size_t i = 0; i < varInfo.size(); ++i) {
            vnames[i] = varInfo[i].getName();
        }
        std::vector<std::string> eqnames(eqInfo.size());
        for (size_t i = 0; i < eqInfo.size(); ++i) {
            if(eqInfo[i].isExplicit()) {
                CPPADCG_ASSERT_UNKNOWN(eqInfo[i].getAssignedVarIndex() >= 0);
                eqnames[i] = "d" + varInfo[eqInfo[i].getAssignedVarIndex()].getName() + "dt";
            } else {
                eqnames[i] = "res[" + std::to_string(i) + "]";
            }
        }

        printModel(out, fun, vnames, eqnames);
    }

    /**
     * Prints out a DAE model to the standard output.
     *
     * @param fun  The taped model
     * @param indepNames  The independent variable names
     * @param depNames  The dependent variable names
     */
    inline void printModel(std::ostream& out,
                           ADFun<CG<Base> >& fun,
                           const std::vector<std::string>& indepNames,
                           const std::vector<std::string>& depNames = std::vector<std::string>()) const {
        using std::vector;

        CPPADCG_ASSERT_UNKNOWN(fun.Domain() == indepNames.size() || fun.Domain() == indepNames.size() + 1); // with or without time

        CodeHandler<Base> handler;

        vector<CGBase> indep0(fun.Domain());
        handler.makeVariables(indep0);

        vector<CGBase> dep0 = forward0(fun, indep0);

        LanguageC<double> langC("double");

        /**
         * generate the source code
         */
        LangCCustomVariableNameGenerator<double> nameGen(depNames, indepNames, "res");

        std::ostringstream code;
        handler.generateCode(code, langC, dep0, nameGen);
        out << "\n" << code.str() << std::endl;
    }

    inline void printDot(std::ostream& out) const {
        out << "digraph {\n";
        out << "   overlap=false\n";
        out << "   rankdir=LR\n";
        out << "   node [style=filled, fillcolor=\"#bdcef5\", color=\"#17128e\"]\n";
        out << "   edge [splines=false, dir=none]\n";

        // variables
        out << "   subgraph variables {\n";
        out << "      rank=min\n";
        for (const Vnode<Base>* j : vnodes_) {
            if(!j->isDeleted()) {
                out << "      v" << j->index() << " [label=\"" << j->name() << "\"";
                if (j->isColored())
                    out << ", color=\"#17c68e\"";
                out << "]\n";
            }
        }
        out << "   }\n";

        // equations
        out << "   subgraph equations {\n";
        out << "      rank=max\n";
        for (const Enode<Base>* i : enodes_) {
            out << "      e" << i->index() << " [label=\"" << i->name() << "\"";
            if (i->isColored())
                out << ", color=\"#17c68e\"";
            out << "]\n";
        }
        out << "   }\n";

        // derivatives of equations
        out << "   subgraph eq_derivatives {\n";
        out << "      edge[dir=forward, color=grey]\n";
        for (const Enode<Base>* i : enodes_) {
            if (i->derivative() != nullptr && i->derivativeOf() == nullptr) {
                while (i->derivative() != nullptr) {
                    out << "      e" << i->index() << ":e -> e" << i->derivative()->index() << ":e\n";
                    i = i->derivative();
                }
            }
        }
        out << "   }\n";

        // derivatives of variables
        out << "   subgraph var_derivatives {\n";
        out << "      edge[dir=forward, color=grey]\n";
        for (const Vnode<Base>* j : vnodes_) {
            if (!j->isDeleted() && j->derivative() != nullptr && (j->antiDerivative() == nullptr || j->antiDerivative()->isDeleted())) {
                if (!j->derivative()->isDeleted()) {
                    while (j->derivative() != nullptr && !j->derivative()->isDeleted()) {
                        out << "      v" << j->index() << ":w -> v" << j->derivative()->index() << ":w\n";
                        j = j->derivative();
                    }
                }
            }
        }
        out << "   }\n";

        // edges
        for (const Enode<Base>* i : enodes_) {
            bool added = false;
            for (const Vnode<Base>* j : i->originalVariables()) {
                if (!j->isDeleted() && j->assignmentEquation() != i) {
                    if(!added) {
                        out << "   ";
                        added = true;
                    }
                    out << "e" << i->index() << " -> v" << j->index() << "  ";
                }
            }
            if (added)
                out << "\n";
        }

        out << "   subgraph assigned {\n";
        out << "      edge[color=blue,penwidth=3.0,style=dashed]\n";
        for (const Enode<Base>* i : enodes_) {
            bool added = false;

            for (const Vnode<Base>* j : i->originalVariables()) {
                if (!j->isDeleted() && j->assignmentEquation() == i) {
                    if(!added) {
                        out << "      ";
                        added = true;
                    }
                    out << "e" << i->index() << " -> v" << j->index() << "  ";
                }
            }

            if (added)
                out << "\n";
        }

        out << "   }\n";
        out << "}\n";
    }

    template<class VectorCGB>
    inline VectorCGB forward0(ADFun<CGBase>& fun,
                              const VectorCGB& indep0) const {

        if (preserveNames_) {
            // stream buffer is used to reload names saved with CppAD::PrintFor()
            OperationNodeNameStreambuf<double> b;
            std::ostream out(&b);

            return fun.Forward(0, indep0, out);
        } else {
            return fun.Forward(0, indep0);
        }
    }

    static inline std::vector<int> determineVariableDiffOrder(const std::vector<DaeVarInfo>& varInfo) {
        size_t n = varInfo.size();
        // determine the order of each time derivative
        std::vector<int> derivOrder(n, 0);
        for (size_t dj = 0; dj < n; dj++) {
            size_t j0;
            derivOrder[dj] = determineVariableDiffOrder(varInfo, dj, j0);
        }

        return derivOrder;
    }

    static inline int determineVariableDiffOrder(const std::vector<DaeVarInfo>& varInfo, size_t index, size_t& j0) {
        int derivOrder = -1;
        j0 = index;
        if (varInfo[index].isFunctionOfIntegrated()) {
            derivOrder = 0;
            while (varInfo[j0].getAntiDerivative() >= 0) {
                CPPADCG_ASSERT_UNKNOWN(j0 < varInfo.size());
                CPPADCG_ASSERT_UNKNOWN(varInfo[j0].isFunctionOfIntegrated());
                derivOrder++;
                j0 = varInfo[j0].getAntiDerivative();
            }
        }

        return derivOrder;
    }

private:
    inline void determineVariableOrder(DaeVarInfo& var) {
        if (var.getAntiDerivative() >= 0) {
            DaeVarInfo& antiD = varInfo_[var.getAntiDerivative()];
            if (antiD.getOriginalAntiDerivative() < 0) {
                determineVariableOrder(antiD);
            }
            var.setOrder(antiD.getOrder() + 1);
            var.setOriginalAntiDerivative(var.getOrder() == 1 ? antiD.getOriginalIndex() : antiD.getOriginalAntiDerivative());
        }
    }
};

template<class Base>
inline std::ostream& operator<<(std::ostream& os,
                                const BipartiteGraph<Base>& g) {
    for (const Enode<Base>* i : g.equations()) {
        for (const Vnode<Base>* j : i->originalVariables()) {
            os << i->name();
            if (j->isDeleted()) {
                os << "~~";
            } else if (j->assignmentEquation() == i) {
                os << "==";
            } else {
                os << "--";
            }
            os << j->name() << " ";
        }
        os << std::endl;
    }

    return os;
}

} // END cg namespace
} // END CppAD namespace

#endif
