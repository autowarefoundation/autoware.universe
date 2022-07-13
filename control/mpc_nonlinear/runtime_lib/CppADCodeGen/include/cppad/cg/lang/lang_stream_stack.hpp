#ifndef CPPAD_CG_LANG_STREAM_STACK_INCLUDED
#define CPPAD_CG_LANG_STREAM_STACK_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
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

#include <ostream>

namespace CppAD {
namespace cg {

namespace _private {

template<class Base>
class LangStreamOperation {
private:
    OperationNode<Base>* _node;
    std::string _text;
public:
    inline LangStreamOperation(OperationNode<Base>& node) :
            _node(&node) {
    }

    inline LangStreamOperation(std::string text) :
            _node(nullptr),
            _text(std::move(text)) {
    }

    OperationNode<Base>* getNode() const {
        return _node;
    }

    const std::string& getText() const {
        return _text;
    }
};

}

/**
 * A cache for sending source code to an output stream which waits until the source for operation nodes
 * are provided.
 *
 * @tparam Base
 */
template<class Base>
class LangStreamStack {
private:
    std::ostream& _out;
    std::forward_list<_private::LangStreamOperation<Base> > _cache;
    typename std::forward_list<_private::LangStreamOperation<Base> >::iterator _it;
public:
    inline LangStreamStack(std::ostream& out) :
            _out(out),
            _it(_cache.before_begin()) {
    }

    inline bool empty() const {
        return _cache.empty();
    }

    inline void clear() {
        _cache.clear();
        _it = _cache.before_begin();
    }

    inline void flush() {
        if (empty())
            return;

        while (!_cache.empty() && _cache.begin()->getNode() == nullptr) {
            _out << _cache.begin()->getText();
            _cache.erase_after(_cache.before_begin());
        }
        _it = _cache.before_begin();
    }

    inline OperationNode<Base>& startNewOperationNode() {
        CPPAD_ASSERT_KNOWN(!_cache.empty(), "Cannot extract an operation node from an empty list")
        CPPAD_ASSERT_KNOWN(_cache.begin()->getNode() != nullptr, "The first element in the list is not an OperationNode")
        OperationNode<Base>* node = _cache.begin()->getNode();
        _cache.erase_after(_cache.before_begin());
        _it = _cache.before_begin();

        return *node;
    }

    friend inline LangStreamStack<Base>& operator<<(LangStreamStack<Base>& lss, std::string text) {
        if (lss._it == lss._cache.before_begin()) {
            lss._out << text;
        } else {
            lss._it = lss._cache.emplace_after(lss._it, std::move(text));
        }
        return lss;
    }

    friend inline LangStreamStack<Base>& operator<<(LangStreamStack<Base>& lss, int i) {
        return (lss << std::to_string(i));
    }

    friend inline LangStreamStack<Base>& operator<<(LangStreamStack<Base>& lss, long int i) {
        return (lss << std::to_string(i));
    }

    friend inline LangStreamStack<Base>& operator<<(LangStreamStack<Base>& lss, long long int i) {
        return (lss << std::to_string(i));
    }

    friend inline LangStreamStack<Base>& operator<<(LangStreamStack<Base>& lss, unsigned int i) {
        return (lss << std::to_string(i));
    }

    friend inline LangStreamStack<Base>& operator<<(LangStreamStack<Base>& lss, long unsigned int i) {
        return (lss << std::to_string(i));
    }

    friend inline LangStreamStack<Base>& operator<<(LangStreamStack<Base>& lss, long long unsigned int i) {
        return (lss << std::to_string(i));
    }

    friend inline LangStreamStack<Base>& operator<<(LangStreamStack<Base>& lss, char text) {
        return (lss << std::string(1, text));
    }

    friend inline LangStreamStack<Base>& operator<<(LangStreamStack<Base>& lss, OperationNode<Base>& node) {
        lss._it = lss._cache.emplace_after(lss._it, node);

        return lss;
    }
};

} // END cg namespace
} // END CppAD namespace

#endif
