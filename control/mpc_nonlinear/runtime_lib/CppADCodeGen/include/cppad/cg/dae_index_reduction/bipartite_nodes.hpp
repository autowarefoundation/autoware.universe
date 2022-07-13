#ifndef CPPAD_CG_BIPARTITE_NODES_INCLUDED
#define CPPAD_CG_BIPARTITE_NODES_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
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

#include <assert.h>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>

namespace CppAD {
namespace cg {

/**
 * Bipartite graph node
 */
template<class Base>
class BiPGraphNode {
protected:
    size_t index_; // location of node
    bool colored_; // node visited
public:

    inline BiPGraphNode(size_t index) :
        index_(index),
        colored_(false) {
    }

    inline void color(std::ostream& out = std::cout,
                      Verbosity verbosity = Verbosity::None) {
        colored_ = true;

        if (verbosity >= Verbosity::High)
            out << "      Colored " << nodeType() << " " << name() << "\n";
    }

    inline void uncolor() {
        colored_ = false;
    }

    inline bool isColored() const {
        return colored_;
    }

    inline size_t index() const {
        return index_;
    }

    inline void setIndex(size_t index) {
        index_ = index;
    }

    virtual const std::string& name() const = 0;

    virtual std::string nodeType() = 0;

    inline virtual ~BiPGraphNode() {
    }
};

template<class Base>
class Vnode; // forward declaration

/**
 * Equation nodes
 */
template<class Base>
class Enode : public BiPGraphNode<Base> {
protected:
    static const std::string TYPE;
    /**
     * Original variables present in this equation.
     * A vector is used instead of a set to ensure reproducibility.
     */
    std::vector<Vnode<Base>*> vnodes_orig_;
    /**
     * Variables present in this equation which where not deleted.
     * A vector is used instead of a set to ensure reproducibility.
     */
    std::vector<Vnode<Base>*> vnodes_;
    /**
     * the differentiated equation used to produce this one
     *  (B in Pantelides algorithm)
     */
    Enode<Base>* differentiation_;
    /**
     * Original equation which was differentiated
     */
    Enode<Base>* differentiationOf_;
    /**
     * 
     */
    Vnode<Base>* assign_;
    /**
     * A name for the equation
     */
    std::string name_;
public:

    inline Enode(size_t index,
                 const std::string& name = "") :
        BiPGraphNode<Base>(index),
        differentiation_(nullptr),
        differentiationOf_(nullptr),
        assign_(nullptr),
        name_(name.empty()? ("Eq" + std::to_string(index)) : name) {
    }

    inline Enode(size_t index,
                 Enode<Base>* differentiationOf) :
        BiPGraphNode<Base>(index),
        differentiation_(nullptr),
        differentiationOf_(differentiationOf),
        assign_(nullptr),
        name_("Diff(" + differentiationOf->name() + ")") {
        differentiationOf_->setDerivative(this);
    }

    inline virtual ~Enode() {
    }

    inline const std::vector<Vnode<Base>*>& variables() const {
        return vnodes_;
    }

    inline const std::vector<Vnode<Base>*>& originalVariables() const {
        return vnodes_orig_;
    }

    inline void addVariable(Vnode<Base>* j) {
        if (std::find(vnodes_orig_.begin(), vnodes_orig_.end(), j) == vnodes_orig_.end()) {
            vnodes_orig_.push_back(j);
            if (!j->isDeleted()) {
                vnodes_.push_back(j);
                j->addEquation(this);
            }
        }
    }

    inline Vnode<Base>* assignmentVariable() const {
        return assign_;
    }

    inline void setAssigmentVariable(Vnode<Base>& j) {
        assign_ = &j;
    }

    /**
     * @return the equation that was derived by differentiating this 
     * equation
     */
    inline Enode<Base>* derivative() const {
        return differentiation_;
    }

    inline Enode<Base>* derivativeOf() const {
        return differentiationOf_;
    }

    inline Enode<Base>* originalEquation() {
        if (differentiationOf_ == nullptr) {
            return this;
        } else {
            return differentiationOf_->originalEquation();
        }
    }

    inline void deleteNode(Vnode<Base>* j) {
        auto it = std::find(vnodes_.begin(), vnodes_.end(), j);
        if (it != vnodes_.end())
            vnodes_.erase(it);
    }

    inline void setDerivative(Enode<Base>* difEq) {
        differentiation_ = difEq;
    }

    virtual const std::string& name() const {
        return name_;
    }

    virtual std::string nodeType() {
        return TYPE;
    }
};

template<class Base>
inline std::ostream& operator <<(std::ostream& os, const Enode<Base>& i) {
    if (i.derivativeOf() != nullptr) {
        os << "Diff(" << *i.derivativeOf() << ")";
    } else {
        os << "Equation " << i.name() << " (" << i.index() << ")";
    }

    return os;
}

template<class Base>
const std::string Enode<Base>::TYPE = "Equation";

/**
 * Variable nodes
 */
template<class Base>
class Vnode : public BiPGraphNode<Base> {
protected:
    static const std::string TYPE;
    /**
     * 
     */
    bool deleted_;
    /**
     * Whether or not this variable is time dependent
     */
    bool parameter_;
    /**
     * Equations that use this variable.
     * A vector is used instead of a set to ensure reproducibility.
     */
    std::vector<Enode<Base>*> enodes_;
    /**
     * 
     */
    Enode<Base>* assign_;
    /**
     *  the time derivative variable of this variable
     *  (A in Pantelides algorithm)
     */
    Vnode<Base>* derivative_;
    /**
     *  the variable which was differentiated to create this one
     */
    Vnode<Base>* const antiDerivative_;
    /**
     * The index in the tape
     */
    size_t tapeIndex_;
    /**
     * name
     */
    std::string name_;

public:

    inline Vnode(size_t index,
                 int tapeIndex, 
                 const std::string& name) :
        BiPGraphNode<Base>(index),
        deleted_(false),
        parameter_(false),
        assign_(nullptr),
        derivative_(nullptr),
        antiDerivative_(nullptr),
        tapeIndex_(tapeIndex),
        name_(name) {

    }

    inline Vnode(size_t index,
                 size_t tapeIndex,
                 Vnode<Base>* derivativeOf,
                 const std::string& name = "") :
        BiPGraphNode<Base>(index),
        deleted_(false),
        parameter_(false),
        assign_(nullptr),
        derivative_(nullptr),
        antiDerivative_(derivativeOf),
        tapeIndex_(tapeIndex),
        name_(name.empty() ? "d" + derivativeOf->name() + "dt" : name) {
        CPPADCG_ASSERT_UNKNOWN(antiDerivative_ != nullptr);

        antiDerivative_->setDerivative(this);
    }

    inline virtual ~Vnode() {
    }

    inline virtual const std::string& name() const {
        return name_;
    }

    inline size_t tapeIndex() const {
        return tapeIndex_;
    }

    inline void setTapeIndex(size_t tapeIndex) {
        tapeIndex_ = tapeIndex;
    }

    inline std::vector<Enode<Base>*>& equations() {
        return enodes_;
    }

    inline const std::vector<Enode<Base>*>& equations() const {
        return enodes_;
    }

    /**
     * @return the time derivative variable
     */
    inline Vnode<Base>* derivative() const {
        return derivative_;
    }

    /**
     * @return the variable which was differentiated to create this one
     */
    inline Vnode<Base>* antiDerivative() const {
        return antiDerivative_;
    }

    inline Vnode<Base>* originalVariable() {
        if (antiDerivative_ == nullptr) {
            return this;
        } else {
            return antiDerivative_->originalVariable();
        }
    }

    inline Vnode<Base>* originalVariable(size_t origVarSize) {
        if (antiDerivative_ == nullptr || this->index_ < origVarSize) {
            return this;
        } else {
            return antiDerivative_->originalVariable();
        }
    }

    inline bool isDeleted() const {
        return deleted_;
    }

    inline void makeParameter(std::ostream& out = std::cout,
                              Verbosity verbosity = Verbosity::None) {
        parameter_ = true;
        deleteNode(out, verbosity);
    }

    inline bool isParameter() const {
        return parameter_;
    }

    inline void deleteNode(std::ostream& out = std::cout,
                           Verbosity verbosity = Verbosity::None) {
        if (verbosity >= Verbosity::High)
            out << "Deleting " << *this << "\n";

        deleted_ = true;
        for (Enode<Base>* i : enodes_) {
            i->deleteNode(this);
        }
        enodes_.clear();
    }

    inline Enode<Base>* assignmentEquation() const {
        return assign_;
    }

    inline void setAssignmentEquation(Enode<Base>& i,
                                      std::ostream& out = std::cout,
                                      Verbosity verbosity = Verbosity::None) {
        if (verbosity >= Verbosity::High)
            out << "      Assigning " << *this << " to " << i << "\n";

        assign_ = &i;
        i.setAssigmentVariable(*this);
    }

    virtual std::string nodeType() {
        return TYPE;
    }

    inline void setDerivative(Vnode<Base>* div) {
        derivative_ = div;
    }

    unsigned int order() const {
        if (antiDerivative_ == nullptr) {
            return 0u;
        } else {
            return antiDerivative_->order() + 1u;
        }
    }

protected:

    inline void addEquation(Enode<Base>* i) {
        if (!deleted_) {
            CPPADCG_ASSERT_UNKNOWN(std::find(enodes_.begin(), enodes_.end(), i) == enodes_.end());
            enodes_.push_back(i);
        }
    }

    friend class Enode<Base>;
};

template<class Base>
inline std::ostream& operator <<(std::ostream& os, const Vnode<Base>& j) {
    if (j.antiDerivative() != nullptr) {
        os << "Diff(" << *j.antiDerivative() << ")";
    } else {
        os << "Variable " << j.name();
    }
    return os;
}

template<class Base>
const std::string Vnode<Base>::TYPE = "Variable";

} // END cg namespace
} // END CppAD namespace

#endif