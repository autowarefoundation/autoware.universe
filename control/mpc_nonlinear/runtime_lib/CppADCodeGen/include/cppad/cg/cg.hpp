#ifndef CPPAD_CG_CG_INCLUDED
#define CPPAD_CG_CG_INCLUDED
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

namespace CppAD {
namespace cg {

/**
 * The base data type used to create models.
 * It can represent either the result of a symbolic operation or a constant
 * parameter value.
 *
 * @author Joao Leal
 */
template<class Base>
class CG {
private:
    /**
     * A node which represents the result from an operation.
     * It must be defined for variables and null for parameters.
     */
    OperationNode<Base>* node_;
    /**
     * A constant value which must be defined for parameters.
     * Its definition is optional for variables.
     */
    std::unique_ptr<Base> value_;

public:
    /**
     * Default constructor (creates a parameter with a zero value)
     */
    inline CG();

    /**
     * Creates a variable resulting from the evaluation this node
     *
     * @param node The operation node.
     */
    inline CG(OperationNode<Base>& node);

    /**
     * Copy constructor
     */
    inline CG(const CG<Base>& orig);

    /**
     * Move constructor
     */
    inline CG(CG<Base>&& orig);

    /**
     * Copy assignment operator
     */
    inline CG& operator=(const CG<Base>& rhs);

    /**
     * Move assignment operator
     */
    inline CG& operator=(CG<Base>&& rhs);

    /**
     * Creates a parameter with the provided value
     *
     * @param val The parameter value
     */
    inline CG(const Base& val);

    /**
     * Assignment operator to a parameter value
     *
     * @param rhs The parameter value
     */
    inline CG& operator=(const Base& rhs);

    // destructor
    virtual ~CG();

    /**
     * @return The code handler that owns the OperationNode when it is a
     *         variable, null when it is a parameter.
     */
    inline CodeHandler<Base>* getCodeHandler() const;

    // variable classification methods
    /**
     * Determines if it represents the result from a symbolic operation
     * which is registered in operation graph of a CodeHandler.
     *
     * @return true if it represents the result from a symbolic operation
     */
    inline bool isVariable() const;

    /**
     * Determines if it a constant parameter which is not the result of
     * a symbolic operation.
     *
     * @return true if it represents a constant parameter.
     */
    inline bool isParameter() const;

    /**
     * Determines if there is value defined.
     * Parameters must have a value defined while variable can optionally
     * define it.
     *
     * @return true if there is a value defined
     */
    inline bool isValueDefined() const;

    /**
     * Provides the current numerical value
     *
     * @throws CGException if a value is not defined
     */
    inline const Base& getValue() const;

    /**
     * Defines a value which can alter the resulting model if this object is
     * a parameter used as an argument to symbolic operations.
     * Variables can also optionally define a value however there will be
     * no impact in the resulting model.
     */
    inline void setValue(const Base& val);

    inline bool isIdenticalZero() const;
    inline bool isIdenticalOne() const;

    inline OperationNode<Base>* getOperationNode() const;

    // computed assignment operators
    inline CG<Base>& operator+=(const CG<Base>& right);
    inline CG<Base>& operator-=(const CG<Base>& right);
    inline CG<Base>& operator*=(const CG<Base>& right);
    inline CG<Base>& operator/=(const CG<Base>& right);
    inline CG<Base>& operator+=(const Base& right);
    inline CG<Base>& operator-=(const Base& right);
    inline CG<Base>& operator*=(const Base& right);
    inline CG<Base>& operator/=(const Base& right);

    template< class T>
    inline CG<Base>& operator+=(const T &right);
    template<class T>
    inline CG<Base>& operator-=(const T &right);
    template<class T>
    inline CG<Base>& operator/=(const T &right);
    template<class T>
    inline CG<Base>& operator*=(const T &right);

    // unary operators
    inline CG<Base> operator+() const;
    inline CG<Base> operator-() const;
protected:
    /**
     * Creates a variable/parameter from an existing argument
     *
     * @param arg An argument that may be a parameter or a variable.
     *            (the node is assumed to already be managed by the handler)
     */
    inline CG(const Argument<Base>& arg);

    //
    inline void makeParameter(const Base& b);

    inline void makeVariable(OperationNode<Base>& operation);

    inline void makeVariable(OperationNode<Base>& operation,
                             std::unique_ptr<Base>& value);

    // creating an argument out of this node
    inline Argument<Base> argument() const;

    /***************************************************************************
     *                               friends
     **************************************************************************/

    friend class CodeHandler<Base>;
    friend class CGAbstractAtomicFun<Base>;
    friend class Loop<Base>;
    friend class LoopModel<Base>;

    /**
     * arithmetic binary operators
     */
    friend CG<Base> CppAD::cg::operator+ <Base>(const CG<Base>& left, const CG<Base>& right);
    friend CG<Base> CppAD::cg::operator- <Base>(const CG<Base>& left, const CG<Base>& right);
    friend CG<Base> CppAD::cg::operator* <Base>(const CG<Base>& left, const CG<Base>& right);
    friend CG<Base> CppAD::cg::operator/ <Base>(const CG<Base>& left, const CG<Base>& right);

    /**
     * order determining functions
     */
    friend bool GreaterThanZero <Base> (const CG<Base>& x);
    friend bool GreaterThanOrZero <Base> (const CG<Base>& x);
    friend bool LessThanZero <Base> (const CG<Base>& x);
    friend bool LessThanOrZero <Base> (const CG<Base>& x);
    friend bool abs_geq <Base> (const CG<Base>& x, const CG<Base>& y);

    // EqualOpSeq function
    friend bool EqualOpSeq <Base> (const CG<Base>& u, const CG<Base>& v);

    // NearEqual function
    friend bool NearEqual <Base> (const CG<Base>& x, const CG<Base>& y, const Base& r, const Base& a);
    friend bool NearEqual <Base> (const Base& x, const CG<Base>& y, const Base& r, const Base& a);
    friend bool NearEqual <Base> (const CG<Base>& x, const Base& y, const Base& r, const Base& a);

    // CondExp function
    friend CG<Base> CondExp<Base>(CGOpCode op,
            const CG<Base>& left,
            const CG<Base>& right,
            const CG<Base>& trueCase,
            const CG<Base>& falseCase,
            bool (*compare)(const Base&, const Base&));

    friend CG<Base> sign<Base>(const CG<Base>& x);

    /**
     * math functions
     */
    friend CG<Base> pow<Base>(const CG<Base> &x, const CG<Base> &y);
    friend CG<Base> abs<Base>(const CG<Base>& var);
    friend CG<Base> fabs<Base>(const CG<Base>& var);
    friend CG<Base> acos<Base>(const CG<Base>& var);
    friend CG<Base> asin<Base>(const CG<Base>& var);
    friend CG<Base> atan<Base>(const CG<Base>& var);
    friend CG<Base> cos<Base>(const CG<Base>& var);
    friend CG<Base> cosh<Base>(const CG<Base>& var);
    friend CG<Base> exp<Base>(const CG<Base>& var);
    friend CG<Base> log<Base>(const CG<Base>& var);
    friend CG<Base> sin<Base>(const CG<Base>& var);
    friend CG<Base> sinh<Base>(const CG<Base>& var);
    friend CG<Base> sqrt<Base>(const CG<Base>& var);
    friend CG<Base> tan<Base>(const CG<Base>& var);
    friend CG<Base> tanh<Base>(const CG<Base>& var);

#if CPPAD_USE_CPLUSPLUS_2011
    // c++11 functions
    friend CG<Base> erf<Base>(const CG<Base>& var); // error function
    friend CG<Base> erfc<Base>(const CG<Base>& var); // complementary error function
    friend CG<Base> asinh<Base>(const CG<Base>& var); // inverse hyperbolic sin
    friend CG<Base> acosh<Base>(const CG<Base>& var); // inverse hyperbolic cosine
    friend CG<Base> atanh<Base>(const CG<Base>& var); // inverse hyperbolic tangent
    friend CG<Base> expm1<Base>(const CG<Base>& var); // exponential of x minus one
    friend CG<Base> log1p<Base>(const CG<Base>& var); // logarithm of one plus x
#endif
};


/**
 * Overrides the default behaviour of operator<<(std::ostream& os, const CG<Base>& c).
 * It can be used recover OperationNode names.
 *
 * @todo replace this struct with a template variable once C++14 is used
 */
template<class Base>
struct CGOStreamFunc {
    static thread_local std::function<std::ostream& (std::ostream&, const CG<Base>&)> FUNC;
};

template<class Base>
thread_local std::function<std::ostream& (std::ostream&, const CG<Base>&)> CGOStreamFunc<Base>::FUNC = nullptr;

/**
 * Output stream operator for CG objects.
 * Default behavior can be overridden using CGOStreamFunc::FUN.
 */
template<class Base>
inline std::ostream& operator<<(
        std::ostream& os, //< stream to write to
        const CG<Base>& v//< vector that is output
        ) {
    if(CGOStreamFunc<Base>::FUNC != nullptr) {
        return CGOStreamFunc<Base>::FUNC(os, v);
    }

    if (v.isParameter()) {
        os << v.getValue();
    } else {
        os << *v.getOperationNode();
        if (v.isValueDefined()) {
            os << " (" << v.getValue() << ")";
        }
    }
    return os;
}

template<class Base>
inline std::ostringstream& operator<<(
        std::ostringstream& os, //< steam to write the vector to
        const CG<Base>& v//< vector that is output
        ) {
    if (v.isParameter()) {
        os << v.getValue();
    } else {
        os << *v.getOperationNode();
        if (v.isValueDefined()) {
            os << " (" << v.getValue() << ")";
        }
    }
    return os;
}

template<class Base>
inline std::istream& operator>>(
        std::istream& is, //< stream to load a parameter value
        CG<Base>& v//< the variable that will be assign the value
        ) {
    Base value;
    is >> value;
    v = value;
    return is;
}

} // END cg namespace

template <class Base>
int Integer(const CppAD::cg::CG<Base>& x) {
    if (x.isValueDefined()) {
        return Integer(x.getValue());
    } else {
        CppAD::ErrorHandler::Call(false, __LINE__, __FILE__, "Integer()", "No value defined");
        return 0;
    }
}

} // END CppAD namespace

#endif
