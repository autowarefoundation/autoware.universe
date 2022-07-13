#ifndef CPPAD_CG_OPERATION_NODE_NAME_STREAMBUF_INCLUDED
#define CPPAD_CG_OPERATION_NODE_NAME_STREAMBUF_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
 *    Copyright (C) 2018 Joao Leal
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
 * A stream buffer used to recover OperationNode names defined using
 * CppAD::PrintFor.
 *
 * @note There can only be one at a time.
 * @note It must be used in the same thread it was created in!
 *
 * @author Joao Leal
 */
template<class Base>
class OperationNodeNameStreambuf : public std::streambuf {
private:
    using char_type = typename std::streambuf::char_type;
    using int_type = typename std::streambuf::int_type;
    using pos_type = typename std::streambuf::pos_type;
private:
    thread_local static OperationNodeNameStreambuf<Base>* BUF;
private:
    OperationNode <Base>* node_;
public:
    OperationNodeNameStreambuf() :
            node_(nullptr) {
        if (BUF != nullptr) {
            throw CGException("Only one OperationNodeNameStreambuf can exist at a time in each thread");
        }
        if (CGOStreamFunc<Base>::FUNC != nullptr) {
            throw CGException("CGOStreamFunc<Base>::FUNC already defined in this thread");
        }
        BUF = this;
        CGOStreamFunc<Base>::FUNC = registerNode;
    }

    virtual ~OperationNodeNameStreambuf() {
        BUF = nullptr;
        CGOStreamFunc<Base>::FUNC = nullptr;
    }

    std::streamsize xsputn(const char_type* s,
                           std::streamsize n) override {
        if (node_ != nullptr && n > 0) {
            node_->setName(std::string(s, n));
            node_ = nullptr;
        }
        return n;
    }

    //int_type overflow(int_type c) override {
    //    return traits_type::eof();
    //}

private:
    static std::ostream& registerNode(std::ostream& os,
                                      const CG<Base>& c) {
        if(c.isVariable()) {
            BUF->node_ = c.getOperationNode();
        }
        return os;
    }
};

template<class Base>
thread_local OperationNodeNameStreambuf<Base>* OperationNodeNameStreambuf<Base>::BUF = nullptr;

} // END cg namespace
} // END CppAD namespace

#endif
