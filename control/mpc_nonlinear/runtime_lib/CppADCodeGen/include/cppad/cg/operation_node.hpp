#ifndef CPPAD_CG_EXPRESSION_NODE_INCLUDED
#define CPPAD_CG_EXPRESSION_NODE_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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

namespace CppAD {
namespace cg {

/**
 * An operation node.
 *
 * @author Joao Leal
 */
template<class Base>
class OperationNode {
    friend class CodeHandler<Base>;
public:
    using iterator = typename std::vector<Argument<Base> >::iterator;
    using const_iterator = typename std::vector<Argument<Base> >::const_iterator;
    using const_reverse_iterator = typename std::vector<Argument<Base> >::const_reverse_iterator;
    using reverse_iterator = typename std::vector<Argument<Base> >::reverse_iterator;
public:
    static const std::set<CGOpCode> CUSTOM_NODE_CLASS;
private:
    /**
     * the source code handler that own this node
     * (only null for temporary OperationNodes)
     */
    CodeHandler<Base>* handler_;
    /**
     * the operation type represented by this node
     */
    CGOpCode operation_;
    /**
     * additional information/options associated with the operation type
     */
    std::vector<size_t> info_;
    /**
     * arguments required by the operation
     * (empty for independent variables and possibly for the 1st assignment
     *  of a dependent variable)
     */
    std::vector<Argument<Base> > arguments_;
    /**
     * index in the CodeHandler managed nodes array
     */
    size_t pos_;
    /**
     * name for the result of this operation
     */
    std::unique_ptr<std::string> name_;
public:
    /**
     * Changes the current operation type into an Alias.
     * @param other the operation node or value this node is going to reference
     */
    inline void makeAlias(const Argument<Base>& other) {
        CPPADCG_ASSERT_UNKNOWN(CUSTOM_NODE_CLASS.find(operation_) == CUSTOM_NODE_CLASS.end()); // TODO: consider relaxing this check

        operation_ = CGOpCode::Alias;
        arguments_.resize(1);
        arguments_[0] = other;
        name_.reset();
    }

    /**
     * Provides the source code handler that owns this node.
     * It can only be null for temporary nodes.
     *
     * @return a CodeHandler which owns this nodes memory (possibly null)
     */
    inline CodeHandler<Base>* getCodeHandler() const {
        return handler_;
    }

    /**
     * Provides the operation type represented by this node.
     * @return Mathematical operation type which this node is the result of.
     */
    inline CGOpCode getOperationType() const {
        return operation_;
    }

    /**
     * Changes the current operation type.
     * The previous operation information/options might also have to be
     * changed, use getInfo() to change it if required.
     * @param op the new operation type
     * @param arguments the arguments for the new operation
     */
    inline void setOperation(CGOpCode op,
                             const std::vector<Argument<Base> >& arguments = std::vector<Argument<Base> >()) {
        CPPADCG_ASSERT_UNKNOWN(op == operation_ || CUSTOM_NODE_CLASS.find(op) == CUSTOM_NODE_CLASS.end()); // cannot transform into a node with a custom class

        operation_ = op;
        arguments_ = arguments;
    }

    /**
     * Provides the arguments used in the operation represented by this
     * node.
     * @return the arguments for the operation in this node (read-only)
     */
    inline const std::vector<Argument<Base> >& getArguments() const {
        return arguments_;
    }

    /**
     * Provides the arguments used in the operation represented by this
     * node.
     * @return the arguments for the operation in this node
     */
    inline std::vector<Argument<Base> >& getArguments() {
        return arguments_;
    }

    /**
     * Provides additional information used in the operation.
     * @return the additional operation information/options  (read-only)
     */
    inline const std::vector<size_t>& getInfo() const {
        return info_;
    }

    /**
     * Provides additional information used in the operation.
     * @return the additional operation information/options
     */
    inline std::vector<size_t>& getInfo() {
        return info_;
    }

    /**
     * Provide the variable name assigned to this node.
     * @return a variable name for the result of this operation or null if
     *         no name was assigned to this node yet
     */
    inline const std::string* getName() const {
        return name_.get();
    }

    /**
     * Defines a new variable name for this node
     * @param name a variable name
     */
    inline void setName(const std::string& name) {
        if (name_ != nullptr)
            *name_ = name;
        else
            name_.reset(new std::string(name));
    }

    /**
     * Clears any name assigned to this node.
     */
    inline void clearName() {
        name_.reset();
    }

    /**
     * Provides the index in CodeHandler which owns this OperationNode.
     * A value of std::numeric_limits<size_t>::max() means that it is not
     * managed by any CodeHandler.
     * This value can change if its position changes in the CodeHandler.
     *
     * @return the index in the CodeHandler's array of managed nodes
     */
    inline size_t getHandlerPosition() const {
        return pos_;
    }

    // argument iterators

    inline iterator begin() {
        return arguments_.begin();
    }

    inline const_iterator begin() const {
        return arguments_.begin();
    }

    inline iterator end() {
        return arguments_.end();
    }

    inline const_iterator end() const {
        return arguments_.end();
    }

    inline reverse_iterator rbegin() {
        return arguments_.rbegin();
    }

    inline const_reverse_iterator rbegin() const {
        return arguments_.rbegin();
    }

    inline reverse_iterator rend() {
        return arguments_.rend();
    }

    inline const_reverse_iterator rend() const {
        return arguments_.rend();
    }

    inline const_iterator cbegin() const noexcept {
        return arguments_.cbegin();
    }

    inline const_iterator cend() const noexcept {
        return arguments_.cend();
    }

    inline const_reverse_iterator crbegin() const noexcept {
        return arguments_.crbegin();
    }

    inline const_reverse_iterator crend() const noexcept {
        return arguments_.crend();
    }

    inline virtual ~OperationNode() = default;

protected:

    inline OperationNode(const OperationNode& orig) :
        handler_(orig.handler_),
        operation_(orig.operation_),
        info_(orig.info_),
        arguments_(orig.arguments_),
        pos_((std::numeric_limits<size_t>::max)()),
        name_(orig.name_ != nullptr ? new std::string(*orig.name_) : nullptr) {
    }

    inline OperationNode(CodeHandler<Base>* handler,
                         CGOpCode op) :
        handler_(handler),
        operation_(op),
        pos_((std::numeric_limits<size_t>::max)()) {
    }

    inline OperationNode(CodeHandler<Base>* handler,
                         CGOpCode op,
                         const Argument<Base>& arg) :
        handler_(handler),
        operation_(op),
        arguments_ {arg},
        pos_((std::numeric_limits<size_t>::max)()) {
    }

    inline OperationNode(CodeHandler<Base>* handler,
                         CGOpCode op,
                         std::vector<Argument<Base> >&& args) :
        handler_(handler),
        operation_(op),
        arguments_(std::move(args)),
        pos_((std::numeric_limits<size_t>::max)()) {
    }

    inline OperationNode(CodeHandler<Base>* handler,
                         CGOpCode op,
                         std::vector<size_t>&& info,
                         std::vector<Argument<Base> >&& args) :
        handler_(handler),
        operation_(op),
        info_(std::move(info)),
        arguments_(std::move(args)),
        pos_((std::numeric_limits<size_t>::max)()) {
    }

    inline OperationNode(CodeHandler<Base>* handler,
                         CGOpCode op,
                         const std::vector<size_t>& info,
                         const std::vector<Argument<Base> >& args) :
        handler_(handler),
        operation_(op),
        info_(info),
        arguments_(args),
        pos_((std::numeric_limits<size_t>::max)()) {
    }

    inline void setHandlerPosition(size_t pos) {
        pos_ = pos;
    }

public:

    /**
     * Creates a temporary operation node.
     *
     * @warning This node should never be provided to a CodeHandler.
     */
    static std::unique_ptr<OperationNode<Base>> makeTemporaryNode(CGOpCode op,
                                                                  const std::vector<size_t>& info,
                                                                  const std::vector<Argument<Base> >& args) {
        return std::unique_ptr<OperationNode<Base>> (new OperationNode<Base>(nullptr, op, info, args));
    }

protected:
    static inline std::set<CGOpCode> makeCustomNodeClassesSet() noexcept;

};

template<class Base>
inline std::set<CGOpCode> OperationNode<Base>::makeCustomNodeClassesSet() noexcept {
    std::set<CGOpCode> s;
    s.insert(CGOpCode::IndexAssign);
    s.insert(CGOpCode::Index);
    s.insert(CGOpCode::LoopStart);
    s.insert(CGOpCode::LoopEnd);
    s.insert(CGOpCode::Pri);
    return s;
}

template<class Base>
const std::set<CGOpCode> OperationNode<Base>::CUSTOM_NODE_CLASS = makeCustomNodeClassesSet();

template<class Base>
inline std::ostream& operator<<(
        std::ostream& os, //< stream to write to
        const OperationNode<Base>& c) {
    CGOpCode op = c.getOperationType();
    switch (op) {
        case CGOpCode::ArrayCreation:
            os << "new $1[" << c.getArguments().size() << "]";
            break;
        case CGOpCode::SparseArrayCreation:
            os << "new $1[" << c.getInfo()[0] << "]";
            break;
        case CGOpCode::ArrayElement:
            os << "$1[" << c.getInfo()[0] << "]";
            break;
        case CGOpCode::AtomicForward:
            os << "atomicFunction.forward(" << c.getInfo()[0] << ", " << c.getInfo()[1] << ", vx, vy, $1, $2)";
            break;
        case CGOpCode::AtomicReverse:
            os << "atomicFunction.reverse(" << c.getInfo()[0] << ", $1, $2, $3, $4)";
            break;
        case CGOpCode::Sign:
            os << "if($1 > 0) { 1 } else if($1 == 0) { 0 } else { -1 }";
            break;

        default:
            os << op;
    }

    return os;
}

} // END cg namespace
} // END CppAD namespace

#endif
