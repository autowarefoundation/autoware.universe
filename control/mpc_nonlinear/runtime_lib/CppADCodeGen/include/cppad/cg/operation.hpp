#ifndef CPPAD_CG_OPERATION_INCLUDED
#define CPPAD_CG_OPERATION_INCLUDED
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
 * Operation types for OperationNode
 * 
 * @author Joao Leal
 */
enum class CGOpCode {
    Assign,               // a = b
    Abs,                  // abs(variable)
    Acos,                 // acos(variable)
    Acosh,                // acosh(variable)
    Add,                  // a + b
    Alias,                // alias (reference to another operation)
    ArrayCreation,        // dense array {a, b, c ...}
    SparseArrayCreation,  // {a, b, c ...}; {index1, index2, index3, ...};
    ArrayElement,         // x[i]
    Asin,                 // asin(variable)
    Asinh,                // asinh(variable)
    Atan,                 // atan(variable)
    Atanh,                // atanh(variable)
    AtomicForward,        // atomicFunction.forward(q, p, vx, vy, tx, ty)
    AtomicReverse,        // atomicFunction.reverse(p, tx, ty, px, py)
    ComLt,                // result = left < right? trueCase: falseCase
    ComLe,                // result = left <= right? trueCase: falseCase
    ComEq,                // result = left == right? trueCase: falseCase
    ComGe,                // result = left >= right? trueCase: falseCase
    ComGt,                // result = left > right? trueCase: falseCase
    ComNe,                // result = left != right? trueCase: falseCase
    Cosh,                 // cosh(variable)
    Cos,                  // cos(variable)
    Div,                  // a / b
    Erf,                  // erf(variable)
    Erfc,                 // erfc(variable)
    Exp,                  // exp(variable)
    Expm1,                // expm1(variable)
    Inv,                  //                             independent variable
    Log,                  // log(variable)
    Log1p,                // log1p(variable)
    Mul,                  // a * b
    Pow,                  // pow(a,   b)
    Pri,                  // PrintFor(text, parameter or variable, parameter or variable)
    Sign,                 // result = (x > 0)? 1.0:((x == 0)? 0.0:-1)
    Sinh,                 // sinh(variable)
    Sin,                  // sin(variable)
    Sqrt,                 // sqrt(variable)
    Sub,                  // a - b
    Tanh,                 // tanh(variable)
    Tan,                  // tan(variable)
    UnMinus,              // -(a)
    DependentMultiAssign, // operation which associates a dependent variables with loops and regular operations
    DependentRefRhs,      // operation referencing a dependent variable (right hand side only)
    IndexDeclaration,     // an integer index declaration
    Index,                // an integer index
    IndexAssign,          // assignment of an integer index to an index pattern expression
    LoopStart,            // for() {}
    LoopIndexedIndep,     // indexed independent used by a loop
    LoopIndexedDep,       // indexed output for a dependent variable from a loop
    LoopIndexedTmp,       // indexed output for a temporary variable from a loop
    LoopEnd,              // endfor
    TmpDcl,               // marks the beginning of the use of a temporary variable across several scopes (used by LoopIndexedTmp)
    Tmp,                  // reference to a temporary variable defined by TmpDcl
    IndexCondExpr,        // a condition expression which returns a boolean
    StartIf,              // the start of an if statement
    ElseIf,               // else if()
    Else,                 // else
    EndIf,                // end of if
    CondResult,           // assignment inside an if branch
    UserCustom,           // a custom type added by a user which has no direct support in CppADCodeGen
    NumberOp              // total number of operation types
};

inline std::ostream& operator<<(std::ostream& os, const CGOpCode& op) {
    static const char* OpNameTable[] = {
            "$1 = $2",                // Assign
            "abs($1)",                // Abs
            "acos($1)",               // Acos
            "acosh($1)",              // Acosh
            "$1 + $2",                // Add
            "alias($1)",              // Alias
            "new array[size]",        // ArrayCreation
            "new sparseArray[size]",  // SparseArrayCreation
            "array[i]",               // ArrayElement
            "asin($1)",               // Asin
            "asinh($1)",              // Asinh
            "atan($1)",               // Atan
            "atanh($1)",              // Atanh
            "atomicFunction.forward(q, p, vx, vy, tx, ty)", // AtomicForward
            "atomicFunction.reverse(p, tx, ty, px, py)",    // AtomicReverse
            "result = ($1 < $2)? $3 : $4",  // ComLt
            "result = ($1 <= $2)? $3 : $4", // ComLe
            "result = ($1 == $2)? $3 : $4", // ComEq
            "result = ($1 >= $2)? $3 : $4", // ComGe
            "result = ($1 > $2)? $3 : $4",  // ComGt
            "result = ($1 != $2)? $3 : $4", // ComNe
            "cosh($1)",               // Cosh
            "cos($1)",                // Cos
            "$1 / $2",                // Div
            "erf($1)",                // Erf
            "erfc($1)",               // Erfc
            "exp($1)",                // Exp
            "expm1($1)",              // Expm1
            "independent()",          // Inv
            "log($1)",                // Log
            "log1p($1)",              // Log1p
            "$1 * $2",                // Mul
            "pow($1, $2)",            // Pow
            "print($1)",              // Pri
            "sign($1)",               // Sign
            "sinh($1)",               // Sinh
            "sin($1)",                // Sin
            "sqrt($1)",               // Sqrt
            "$1 - $2",                // Sub
            "tanh($1)",               // Tanh
            "tan($1)",                // Tan
            "-($1)",                  // UnMinus
            "dep($1) = ($2) + ...",   // DependentMultiAssign
            "depref($1)",             // DependentRefRhs
            "index declaration",      // IndexDeclaration
            "index",                  // Index
            "index = expression()",   // IndexAssign
            "for",                    // LoopStart
            "loopIndexedIndep",       // LoopIndexedIndep
            "loopIndexedDep",         // LoopIndexedDep
            "loopIndexedTmp",         // LoopIndexedTmp
            "endfor",                 // LoopEnd
            "declare tempVar",        // TmpDcl
            "tempVar",                // Tmp
            "bool(index expression)", // IndexCondExpr
            "if()",                   // StartIf
            "else if()",              // ElseIf
            "else",                   // Else
            "endif",                  // EndIf
            "ifResult =",             // CondResult
            "custom",                 // UserCustom
            "numberOp"
    };
    // check ensuring conversion to size_t is as expected
    CPPADCG_ASSERT_UNKNOWN(size_t(CGOpCode::NumberOp) + 1 == sizeof(OpNameTable)/sizeof(OpNameTable[0]));

    // this test ensures that all indices are within the table
    CPPADCG_ASSERT_UNKNOWN(int(op) >= 0 && size_t(op) < size_t(CGOpCode::NumberOp));
    
    os << OpNameTable[size_t(op)];

    return os;
}

} // END cg namespace
} // END CppAD namespace

#endif