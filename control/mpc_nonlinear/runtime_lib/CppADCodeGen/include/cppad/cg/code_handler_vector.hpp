#ifndef CPPAD_CG_CODE_HANDLER_VECTOR_INCLUDED
#define CPPAD_CG_CODE_HANDLER_VECTOR_INCLUDED
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

// forward declaration
template<class Base>
class CodeHandler;

/**
 * A class for synchronization of data vectors associated with operation nodes
 * managed by a code handler.
 */
template<class Base>
class CodeHandlerVectorSync {
    friend class CodeHandler<Base>;
protected:
    CodeHandler<Base>* handler_;
public:

    inline CodeHandlerVectorSync(CodeHandler<Base>& handler) :
        handler_(&handler) {
        handler_->addVector(this);
    }

    inline CodeHandlerVectorSync(const CodeHandlerVectorSync& orig) :
        handler_(orig.handler_) {
        handler_->addVector(this);
    }

    virtual ~CodeHandlerVectorSync() {
        if (handler_ != nullptr)
            handler_->removeVector(this);
    }

    inline CodeHandler<Base>& getHandler() const {
        return *handler_;
    }

protected:
    /**
     * @param start The index of the first OperationNode that was deleted
     * @param end The index after the last OperationNode that was deleted
     */
    virtual void nodesErased(size_t start,
                             size_t end) = 0;
};

/**
 * A vector for data associated with operation nodes managed by a code handler.
 *
 * @author Joao Leal
 */
template<class Base, class T>
class CodeHandlerVector : public CodeHandlerVectorSync<Base> {
public:
    using iterator = typename std::vector<T>::iterator;
    using const_iterator = typename std::vector<T>::const_iterator;
    using const_reverse_iterator = typename std::vector<T>::const_reverse_iterator;
    using reverse_iterator = typename std::vector<T>::reverse_iterator;
    using reference = typename std::vector<T>::reference;
    using const_reference = typename std::vector<T>::const_reference;
private:
    /**
     * data vector
     */
    std::vector<T> data_;
public:

    inline CodeHandlerVector(CodeHandler<Base>& handler) :
        CodeHandlerVectorSync<Base>(handler) {
    }

    inline CodeHandlerVector(const CodeHandlerVector& orig) :
        CodeHandlerVectorSync<Base>(orig),
        data_(orig.data_) {
    }

    inline void clear() {
        data_.clear();
    }

    inline void adjustSize() {
        size_t s = this->handler_->getManagedNodesCount();
        if (s >= data_.capacity()) {
            data_.reserve(s * 3 / 2 + 1);
        }
        data_.resize(s);
    }

    inline void adjustSize(const OperationNode<Base>& node) {
        CPPADCG_ASSERT_UNKNOWN(node.getCodeHandler() == this->handler_)

        size_t p = node.getHandlerPosition();
        if (p == (std::numeric_limits<size_t>::max)())
            throw CGException("An operation node is not managed by this code handler");

        if (p >= data_.size())
            adjustSize();
    }

    inline reference get(const OperationNode<Base>& node) {
        CPPADCG_ASSERT_UNKNOWN(node.getCodeHandler() == this->handler_)

        size_t p = node.getHandlerPosition();
        if (p == (std::numeric_limits<size_t>::max)())
            throw CGException("An operation node is not managed by this code handler");
        CPPADCG_ASSERT_UNKNOWN(p < data_.size())

        return data_[p];
    }

    inline const_reference get(const OperationNode<Base>& node) const {
        CPPADCG_ASSERT_UNKNOWN(node.getCodeHandler() == this->handler_)

        size_t p = node.getHandlerPosition();
        if (p == (std::numeric_limits<size_t>::max)())
            throw CGException("An operation node is not managed by this code handler");
        CPPADCG_ASSERT_UNKNOWN(p < data_.size())

        return data_[p];
    }

    inline void set(const OperationNode<Base>& node,
                    const T& val) {
        CPPADCG_ASSERT_UNKNOWN(node.getCodeHandler() == this->handler_)

        size_t p = node.getHandlerPosition();
        if (p == (std::numeric_limits<size_t>::max)())
            throw CGException("An operation node is not managed by this code handler");
        CPPADCG_ASSERT_UNKNOWN(p < data_.size())

        data_[node.getHandlerPosition()] = val;
    }

    inline size_t size() const {
        return data_.size();
    }

    inline bool empty() const {
        return data_.empty();
    }

    inline void fill(const T& v) {
        std::fill(data_.begin(), data_.end(), v);
    }

    // iterators

    inline iterator begin() {
        return data_.begin();
    }

    inline const_iterator begin() const {
        return data_.begin();
    }

    inline iterator end() {
        return data_.end();
    }

    inline const_iterator end() const {
        return data_.end();
    }

    inline reverse_iterator rbegin() {
        return data_.rbegin();
    }

    inline const_reverse_iterator rbegin() const {
        return data_.rbegin();
    }

    inline reverse_iterator rend() {
        return data_.rend();
    }

    inline const_reverse_iterator rend() const {
        return data_.rend();
    }

    inline const_iterator cbegin() const noexcept {
        return data_.cbegin();
    }

    inline const_iterator cend() const noexcept {
        return data_.cend();
    }

    inline const_reverse_iterator crbegin() const noexcept {
        return data_.crbegin();
    }

    inline const_reverse_iterator crend() const noexcept {
        return data_.crend();
    }
protected:

    void nodesErased(size_t start,
                     size_t end) override {
        if (start < data_.size()) {
            end = std::min<size_t>(end, data_.size());
            data_.erase(data_.begin() + start, data_.begin() + end);
        }
    }

public:

    /**
     * operators
     */
    inline reference operator[](const OperationNode<Base>& node) {
        return get(node);
    }

    inline const_reference operator[](const OperationNode<Base>& node) const {
        return get(node);
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
