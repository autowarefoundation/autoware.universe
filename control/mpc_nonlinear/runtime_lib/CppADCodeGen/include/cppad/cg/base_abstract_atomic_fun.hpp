#ifndef CPPAD_CG_BASE_ABSTRACT_ATOMIC_FUN_INCLUDED
#define CPPAD_CG_BASE_ABSTRACT_ATOMIC_FUN_INCLUDED
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

namespace CppAD {
namespace cg {

/**
 * Contains some utility methods for atomic functions for the type CG
 * 
 * @author Joao Leal
 */
template <class Base>
class BaseAbstractAtomicFun : public atomic_base<CppAD::cg::CG<Base> > {
public:
    using CGB = CppAD::cg::CG<Base>;
    using Arg = Argument<Base>;
protected:

    /**
     * Creates a new atomic function that is responsible for defining the
     * dependencies to calls of a user atomic function.
     * 
     * @param name The atomic function name.
     */
    explicit BaseAbstractAtomicFun(const std::string& name) :
            atomic_base<CGB>(name) {
        CPPADCG_ASSERT_KNOWN(!name.empty(), "The atomic function name cannot be empty")
    }

public:

    template <class ADVector>
    void operator()(const ADVector& ax,
                    ADVector& ay,
                    size_t id = 0) {
        this->atomic_base<CGB>::operator()(ax, ay, id);
    }

    virtual ~BaseAbstractAtomicFun() = default;

protected:

    static inline void appendAsArguments(typename std::vector<Arg>::iterator begin,
                                         const CppAD::vector<CGB>& tx) {
        std::vector<Arg> arguments(tx.size());
        typename std::vector<Arg>::iterator it = begin;
        for (size_t i = 0; i < arguments.size(); i++, ++it) {
            if (tx[i].isParameter()) {
                *it = Arg(tx[i].getValue());
            } else {
                *it = Arg(*tx[i].getOperationNode());
            }
        }
    }

    static inline OperationNode<Base>* makeArray(CodeHandler<Base>& handler,
                                                 const CppAD::vector<CGB>& tx) {
        std::vector<Arg> arrayArgs = asArguments(tx);
        std::vector<size_t> info; // empty

        return handler.makeNode(CGOpCode::ArrayCreation, info, arrayArgs);
    }

    static inline OperationNode<Base>* makeArray(CodeHandler<Base>& handler,
                                                 const CppAD::vector<CGB>& tx,
                                                 size_t p,
                                                 size_t k) {
        CPPADCG_ASSERT_UNKNOWN(k <= p)
        size_t n = tx.size() / (p + 1);
        std::vector<Arg> arrayArgs(n);
        for (size_t i = 0; i < n; i++) {
            arrayArgs[i] = asArgument(tx[i * (p + 1) + k]);
        }

        return handler.makeNode(CGOpCode::ArrayCreation,{}, arrayArgs);
    }

    static inline OperationNode<Base>* makeZeroArray(CodeHandler<Base>& handler,
                                                     size_t size) {
        CppAD::vector<CGB> tx2(size);
        std::vector<Arg> arrayArgs = asArguments(tx2);

        return handler.makeNode(CGOpCode::ArrayCreation,{}, arrayArgs);
    }

    static inline OperationNode<Base>* makeEmptySparseArray(CodeHandler<Base>& handler,
                                                            size_t size) {
        return handler.makeNode(CGOpCode::SparseArrayCreation,{size}, {}); //empty args
    }

    static inline OperationNode<Base>* makeSparseArray(CodeHandler<Base>& handler,
                                                       const CppAD::vector<CGB>& py,
                                                       size_t p,
                                                       size_t k) {
        size_t p1 = p + 1;
        CPPADCG_ASSERT_UNKNOWN(k < p1)
        size_t n = py.size() / p1;

        std::vector<Arg> arrayArgs;
        std::vector<size_t> arrayIdx(1);
        arrayIdx[0] = n; // array size

        arrayArgs.reserve(py.size() / 3);
        arrayIdx.reserve(1 + py.size() / 3);

        for (size_t i = 0; i < n; i++) {
            if (!py[i * p1 + k].isIdenticalZero()) {
                arrayArgs.push_back(asArgument(py[i * p1 + k]));
                arrayIdx.push_back(i);
            }
        }

        return handler.makeNode(CGOpCode::SparseArrayCreation, arrayIdx, arrayArgs);
    }

    static inline bool isParameters(const CppAD::vector<CGB>& tx) {
        for (size_t i = 0; i < tx.size(); i++) {
            if (!tx[i].isParameter()) {
                return false;
            }
        }
        return true;
    }

    static inline bool isValuesDefined(const CppAD::vector<CGB>& tx) {
        for (size_t i = 0; i < tx.size(); i++) {
            if (!tx[i].isValueDefined()) {
                return false;
            }
        }
        return true;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif