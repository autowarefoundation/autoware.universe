# ifndef CPPAD_CPPAD_IPOPT_SRC_CPPAD_IPOPT_NLP_HPP
# define CPPAD_CPPAD_IPOPT_SRC_CPPAD_IPOPT_NLP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin cppad_ipopt_nlp$$
$dollar @$$
$spell
    libipopt
    namespace
    dir
    cppad
    bool
    doesn't
    nan
    inf
    naninf
    std
    maxiter
    infeasibility
    obj
    const
    optimizer
    cppad_ipopt_nlp.hpp
    fg_info.eval
    retape
    CppAD
    config
    lipopt

$$
$section Nonlinear Programming Using the CppAD Interface to Ipopt$$


$head Deprecated 2012-11-28$$
This interface to Ipopt is deprecated, use $cref ipopt_solve$$ instead.

$head Syntax$$
$codei%# include "cppad_ipopt_nlp.hpp"
%$$
$codei%cppad_ipopt_solution %solution%;
%$$
$codei%cppad_ipopt_nlp %cppad_nlp%(
    %n%, %m%, %x_i%, %x_l%, %x_u%, %g_l%, %g_u%, &%fg_info%, &%solution%
)%$$
$codei%
export LD_LIBRARY_PATH=@LD_LIBRARY_PATH:%ipopt_library_paths%$$

$head Purpose$$
The class $code cppad_ipopt_nlp$$ is used to solve nonlinear programming
problems of the form
$latex \[
\begin{array}{rll}
{\rm minimize}      & f(x)
\\
{\rm subject \; to} & g^l \leq g(x) \leq g^u
\\
                    & x^l  \leq x   \leq x^u
\end{array}
\] $$
This is done using
$href%
    http://www.coin-or.org/projects/Ipopt.xml%
    Ipopt
%$$
optimizer and
$href%
    http://www.coin-or.org/CppAD/%
    CppAD
%$$
Algorithmic Differentiation package.

$head cppad_ipopt namespace$$
All of the declarations for these routines
are in the $code cppad_ipopt$$ namespace
(not the $code CppAD$$ namespace).
For example; $cref/SizeVector/cppad_ipopt_nlp/SizeVector/$$ below
actually denotes the type $code cppad_ipopt::SizeVector$$.

$head ipopt_library_paths$$
If you are linking to a shared version of the Ipopt library,
you may have to add a path to the $code LD_LIBRARY_PATH$$.
You can determine the directory you need to add using the command
$code
    pkg-config ipopt --libs
%$$
The output will have the following form
$codei%
    -L%dir% -lipopt
%$$
You may need to add the directory %dir% to $code LD_LIBRARY_PATH%$$; e.g.,
$codei%
    export LD_LIBRARY_PATH="%dir%:@LD_LIBRARY_PATH"
%$$

$head fg(x)$$
The function $latex fg : \B{R}^n \rightarrow \B{R}^{m+1}$$ is defined by
$latex \[
\begin{array}{rcl}
    fg_0 (x)     & = & f(x)         \\
    fg_1 (x)     & = & g_0 (x)      \\
                 & \vdots &         \\
    fg_m (x)     & = & g_{m-1} (x)
    \end{array}
\] $$

$subhead Index Vector$$
We define an $icode index vector$$ as a vector of non-negative integers
for which none of the values are equal; i.e.,
it is both a vector and a set.
If $latex I$$ is an index vector $latex |I|$$ is used to denote the
number of elements in $latex I$$ and $latex \| I \|$$ is used
to denote the value of the maximum element in $latex I$$.

$subhead Projection$$
Given an index vector $latex J$$ and a positive integer $latex n$$
where $latex n > \| J \|$$, we use $latex J \otimes n $$ for
the mapping $latex ( J \otimes n ) : \B{R}^n \rightarrow \B{R}^{|J|}$$ defined by
$latex \[
    [ J \otimes n ] (x)_j = x_{J(j)}
\] $$
for $latex j = 0 , \ldots |J| - 1$$.

$subhead Injection$$
Given an index vector $latex I$$ and a positive integer $latex m$$
where $latex m > \| I \|$$, we use $latex m \otimes I$$ for
the mapping $latex ( m \otimes I ): \B{R}^{|I|} \rightarrow \B{R}^m$$ defined by
$latex \[
[ m \otimes I ] (y)_i = \left\{ \begin{array}{ll}
y_k & {\rm if} \; i = I(k) \; {\rm for \; some} \;
    k \in \{ 0 , \cdots, |I|-1 \}
\\
0   & {\rm otherwise}
\end{array} \right.
\] $$

$subhead Representation$$
In many applications, each of the component functions of $latex fg(x)$$
only depend on a few of the components of $latex x$$.
In this case, expressing $latex fg(x)$$ in terms of simpler functions
with fewer arguments can greatly reduce the amount of work required
to compute its derivatives.
$pre

$$
We use the functions
$latex r_k : \B{R}^{q(k)} \rightarrow \B{R}^{p(k)}$$
for $latex k = 0 , \ldots , K$$ to express our
representation of $latex fg(x)$$ in terms of simpler functions
as follows
$latex \[
fg(x) = \sum_{k=0}^{K-1} \; \sum_{\ell=0}^{L(k) - 1}
[ (m+1) \otimes I_{k,\ell} ] \; \circ
     \; r_k \; \circ \; [ J_{k,\ell} \otimes n ] \; (x)
\] $$
where $latex \circ$$ represents function composition,
for $latex k = 0 , \ldots , K - 1$$, and $latex \ell = 0 , \ldots , L(k)$$,
$latex I_{k,\ell}$$ and  $latex J_{k,\ell}$$ are index vectors with
$latex | J_{k,\ell} | = q(k)$$,
$latex \| J_{k,\ell} \| < n$$,
$latex | I_{k,\ell} | = p(k)$$, and
$latex \| I_{k,\ell} \| \leq m$$.

$head Simple Representation$$
In the simple representation,
$latex r_0 (x) = fg(x)$$,
$latex K = 1$$,
$latex q(0) = n$$,
$latex p(0) = m+1$$,
$latex L(0) = 1$$,
$latex I_{0,0} = (0 , \ldots , m)$$,
and $latex J_{0,0} = (0 , \ldots , n-1)$$.

$head SizeVector$$
The type $codei SizeVector$$ is defined by the
$codei cppad_ipopt_nlp.hpp$$ include file to be a
$cref SimpleVector$$ class with elements of type
$code size_t$$.

$head NumberVector$$
The type $codei NumberVector$$ is defined by the
$codei cppad_ipopt_nlp.hpp$$ include file to be a
$cref SimpleVector$$ class with elements of type
$code Ipopt::Number$$.

$head ADNumber$$
The type $codei ADNumber$$ is defined by the
$codei cppad_ipopt_nlp.hpp$$ include file to be a
an AD type that can be used to compute derivatives.

$head ADVector$$
The type $codei ADVector$$ is defined by the
$codei cppad_ipopt_nlp.hpp$$ include file to be a
$cref SimpleVector$$ class with elements of type
$code ADNumber$$.

$head n$$
The argument $icode n$$ has prototype
$codei%
    size_t %n%
%$$
It specifies the dimension of the argument space;
i.e., $latex x \in \B{R}^n$$.

$head m$$
The argument $icode m$$ has prototype
$codei%
    size_t %m%
%$$
It specifies the dimension of the range space for $latex g$$;
i.e., $latex g : \B{R}^n \rightarrow \B{R}^m$$.

$head x_i$$
The argument $icode x_i$$ has prototype
$codei%
    const NumberVector& %x_i%
%$$
and its size is equal to $latex n$$.
It specifies the initial point where Ipopt starts the optimization process.

$head x_l$$
The argument $icode x_l$$ has prototype
$codei%
    const NumberVector& %x_l%
%$$
and its size is equal to $latex n$$.
It specifies the lower limits for the argument in the optimization problem;
i.e., $latex x^l$$.

$head x_u$$
The argument $icode x_u$$ has prototype
$codei%
    const NumberVector& %x_u%
%$$
and its size is equal to $latex n$$.
It specifies the upper limits for the argument in the optimization problem;
i.e., $latex x^u$$.

$head g_l$$
The argument $icode g_l$$ has prototype
$codei%
    const NumberVector& %g_l%
%$$
and its size is equal to $latex m$$.
It specifies the lower limits for the constraints in the optimization problem;
i.e., $latex g^l$$.

$head g_u$$
The argument $icode g_u$$ has prototype
$codei%
    const NumberVector& %g_u%
%$$
and its size is equal to $latex n$$.
It specifies the upper limits for the constraints in the optimization problem;
i.e., $latex g^u$$.

$head fg_info$$
The argument $icode fg_info$$ has prototype
$codei%
    %FG_info fg_info%
%$$
where the class $icode FG_info$$ is derived from the
base class $code cppad_ipopt_fg_info$$.
Certain virtual member functions of $icode fg_info$$ are used to
compute the value of $latex fg(x)$$.
The specifications for these member functions are given below:

$subhead fg_info.number_functions$$
This member function has prototype
$codei%
    virtual size_t cppad_ipopt_fg_info::number_functions(void)
%$$
If $icode K$$ has type $code size_t$$, the syntax
$codei%
    %K% = %fg_info%.number_functions()
%$$
sets $icode K$$ to the number of functions used in the
representation of $latex fg(x)$$; i.e., $latex K$$ in
the $cref/representation/cppad_ipopt_nlp/fg(x)/Representation/$$ above.
$pre

$$
The $code cppad_ipopt_fg_info$$ implementation of this function
corresponds to the simple representation mentioned above; i.e.
$icode%K% = 1%$$.

$subhead fg_info.eval_r$$
This member function has the prototype
$codei%
virtual ADVector cppad_ipopt_fg_info::eval_r(size_t %k%, const ADVector& %u%) = 0;
%$$
Thus it is a pure virtual function and must be defined in the
derived class $icode FG_info$$.
$pre

$$
This function computes the value of $latex r_k (u)$$
used in the $cref/representation/cppad_ipopt_nlp/fg(x)/Representation/$$
for $latex fg(x)$$.
If $icode k$$ in $latex \{0 , \ldots , K-1 \}$$ has type $code size_t$$,
$icode u$$ is an $code ADVector$$ of size $icode q(k)$$
and $icode r$$ is an $code ADVector$$ of size $icode p(k)$$
the syntax
$codei%
    %r% = %fg_info%.eval_r(%k%, %u%)
%$$
set $icode r$$ to the vector $latex r_k (u)$$.

$subhead fg_info.retape$$
This member function has the prototype
$codei%
    virtual bool cppad_ipopt_fg_info::retape(size_t %k%)
%$$
If $icode k$$ in $latex \{0 , \ldots , K-1 \}$$ has type $code size_t$$,
and $icode retape$$ has type $code bool$$,
the syntax
$codei%
        %retape% = %fg_info%.retape(%k%)
%$$
sets $icode retape$$ to true or false.
If $icode retape$$ is true,
$code cppad_ipopt_nlp$$ will retape the operation sequence
corresponding to $latex r_k (u)$$ for
every value of $icode u$$.
An $code cppad_ipopt_nlp$$ object
should use much less memory and run faster if $icode retape$$ is false.
You can test both the true and false cases to make sure
the operation sequence does not depend on $icode u$$.
$pre

$$
The $code cppad_ipopt_fg_info$$ implementation of this function
sets $icode retape$$ to true
(while slower it is also safer to always retape).

$subhead fg_info.domain_size$$
This member function has prototype
$codei%
    virtual size_t cppad_ipopt_fg_info::domain_size(size_t %k%)
%$$
If $icode k$$ in $latex \{0 , \ldots , K-1 \}$$ has type $code size_t$$,
and $icode q$$ has type $code size_t$$, the syntax
$codei%
    %q% = %fg_info%.domain_size(%k%)
%$$
sets $icode q$$ to the dimension of the domain space for $latex r_k (u)$$;
i.e., $latex q(k)$$ in
the $cref/representation/cppad_ipopt_nlp/fg(x)/Representation/$$ above.

$pre

$$
The $code cppad_ipopt_h_base$$ implementation of this function
corresponds to the simple representation mentioned above; i.e.,
$latex q = n$$.

$subhead fg_info.range_size$$
This member function has prototype
$codei%
    virtual size_t cppad_ipopt_fg_info::range_size(size_t %k%)
%$$
If $icode k$$ in $latex \{0 , \ldots , K-1 \}$$ has type $code size_t$$,
and $icode p$$ has type $code size_t$$, the syntax
$codei%
    %p% = %fg_info%.range_size(%k%)
%$$
sets $icode p$$ to the dimension of the range space for $latex r_k (u)$$;
i.e., $latex p(k)$$ in
the $cref/representation/cppad_ipopt_nlp/fg(x)/Representation/$$ above.
$pre

$$
The $code cppad_ipopt_h_base$$ implementation of this function
corresponds to the simple representation mentioned above; i.e.,
$latex p = m+1$$.

$subhead fg_info.number_terms$$
This member function has prototype
$codei%
    virtual size_t cppad_ipopt_fg_info::number_terms(size_t %k%)
%$$
If $icode k$$ in $latex \{0 , \ldots , K-1 \}$$ has type $code size_t$$,
and $icode L$$ has type $code size_t$$, the syntax
$codei%
    %L% = %fg_info%.number_terms(%k%)
%$$
sets $icode L$$ to the number of terms in representation
for this value of $icode k$$;
i.e., $latex L(k)$$ in
the $cref/representation/cppad_ipopt_nlp/fg(x)/Representation/$$ above.
$pre

$$
The $code cppad_ipopt_h_base$$ implementation of this function
corresponds to the simple representation mentioned above; i.e.,
$latex L = 1$$.

$subhead fg_info.index$$
This member function has prototype
$codei%
    virtual void cppad_ipopt_fg_info::index(
        size_t %k%, size_t %ell%, SizeVector& %I%, SizeVector& %J%
    )
%$$
The argument
$icode%
    k
%$$
has type $codei size_t$$
and is a value between zero and $latex K-1$$ inclusive.
The argument
$icode%
    ell
%$$
has type $codei size_t$$
and is a value between zero and $latex L(k)-1$$ inclusive.
The argument
$icode%
    I
%$$ is a $cref SimpleVector$$ with elements
of type $code size_t$$ and size greater than or equal to $latex p(k)$$.
The input value of the elements of $icode I$$ does not matter.
The output value of
the first $latex p(k)$$ elements of $icode I$$
must be the corresponding elements of $latex I_{k,ell}$$
in the $cref/representation/cppad_ipopt_nlp/fg(x)/Representation/$$ above.
The argument
$icode%
    J
%$$ is a $cref SimpleVector$$ with elements
of type $code size_t$$ and size greater than or equal to $latex q(k)$$.
The input value of the elements of $icode J$$ does not matter.
The output value of
the first $latex q(k)$$ elements of $icode J$$
must be the corresponding elements of $latex J_{k,ell}$$
in the $cref/representation/cppad_ipopt_nlp/fg(x)/Representation/$$ above.
$pre

$$
The $code cppad_ipopt_h_base$$ implementation of this function
corresponds to the simple representation mentioned above; i.e.,
for $latex i = 0 , \ldots , m$$,
$icode%I%[%i%] = %i%$$,
and  for $latex j = 0 , \ldots , n-1$$,
$icode%J%[%j%] = %j%$$.

$head solution$$
After the optimization process is completed, $icode solution$$ contains
the following information:

$subhead status$$
The $icode status$$ field of $icode solution$$ has prototype
$codei%
    cppad_ipopt_solution::solution_status %solution%.status
%$$
It is the final Ipopt status for the optimizer.
Here is a list of the possible values for the status:

$table
$icode status$$ $cnext Meaning
$rnext
not_defined $cnext
The optimizer did not return a final status to this $code cppad_ipopt_nlp$$
object.
$rnext
unknown $cnext
The status returned by the optimizer is not defined in the Ipopt
documentation for $code finalize_solution$$.
$rnext
success $cnext
Algorithm terminated successfully at a point satisfying the convergence
tolerances (see Ipopt options).
$rnext
maxiter_exceeded $cnext
The maximum number of iterations was exceeded (see Ipopt options).
$rnext
stop_at_tiny_step $cnext
Algorithm terminated because progress was very slow.
$rnext
stop_at_acceptable_point $cnext
Algorithm stopped at a point that was converged,
not to the 'desired' tolerances, but to 'acceptable' tolerances
(see Ipopt options).
$rnext
local_infeasibility $cnext
Algorithm converged to a non-feasible point
(problem may have no solution).
$rnext
user_requested_stop $cnext
This return value should not happen.
$rnext
diverging_iterates $cnext
It the iterates are diverging.
$rnext
restoration_failure $cnext
Restoration phase failed, algorithm doesn't know how to proceed.
$rnext
error_in_step_computation $cnext
An unrecoverable error occurred while Ipopt tried to
compute the search direction.
$rnext
invalid_number_detected $cnext
Algorithm received an invalid number (such as $code nan$$ or $code inf$$)
from the users function $icode%fg_info%.eval%$$ or from the CppAD evaluations
of its derivatives
(see the Ipopt option $code check_derivatives_for_naninf$$).
$rnext
internal_error $cnext
An unknown Ipopt internal error occurred.
Contact the Ipopt authors through the mailing list.
$tend

$subhead x$$
The $code x$$ field of $icode solution$$ has prototype
$codei%
    NumberVector %solution%.x
%$$
and its size is equal to $latex n$$.
It is the final $latex x$$ value for the optimizer.

$subhead z_l$$
The $code z_l$$ field of $icode solution$$ has prototype
$codei%
    NumberVector %solution%.z_l
%$$
and its size is equal to $latex n$$.
It is the final Lagrange multipliers for the
lower bounds on $latex x$$.

$subhead z_u$$
The $code z_u$$ field of $icode solution$$ has prototype
$codei%
    NumberVector %solution%.z_u
%$$
and its size is equal to $latex n$$.
It is the final Lagrange multipliers for the
upper bounds on $latex x$$.

$subhead g$$
The $code g$$ field of $icode solution$$ has prototype
$codei%
    NumberVector %solution%.g
%$$
and its size is equal to $latex m$$.
It is the final value for the constraint function $latex g(x)$$.

$subhead lambda$$
The $code lambda$$ field of $icode solution$$ has prototype
$codei%
    NumberVector %solution%.lambda
%$$
and its size is equal to $latex m$$.
It is the final value for the
Lagrange multipliers corresponding to the constraint function.

$subhead obj_value$$
The $code obj_value$$ field of $icode solution$$ has prototype
$codei%
    Number %solution%.obj_value
%$$
It is the final value of the objective function $latex f(x)$$.


$end
-----------------------------------------------------------------------------
*/
# include <cppad/cppad.hpp>
# include <coin-or/IpIpoptApplication.hpp>
# include <coin-or/IpTNLP.hpp>

/*!
\file cppad_ipopt_nlp.hpp
\brief CppAD interface to Ipopt

\ingroup cppad_ipopt_nlp_cpp
*/

// ---------------------------------------------------------------------------
namespace cppad_ipopt {
// ---------------------------------------------------------------------------

/// A scalar value used to record operation sequence.
typedef CppAD::AD<Ipopt::Number>       ADNumber;
/// A simple vector of values used to record operation sequence
typedef CppAD::vector<ADNumber>        ADVector;
/// A simple vector of size_t values.
typedef CppAD::vector<size_t>          SizeVector;
/// A simple vector of values used by Ipopt
typedef CppAD::vector<Ipopt::Number>   NumberVector;

/*!
Abstract base class user derives from to define the funcitons in the problem.
*/
class cppad_ipopt_fg_info
{
    /// allow cppad_ipopt_nlp class complete access to this class
    friend class cppad_ipopt_nlp;
private:
    /// domain space dimension for the functions f(x), g(x)
    size_t n_;
    /// range space dimension for the function g(x)
    size_t m_;
    /// the cppad_ipopt_nlp constructor uses this method to set n_
    void set_n(size_t n)
    {   n_ = n; }
    /// the cppad_ipopt_nlp constructor uses this method to set m_
    void set_m(size_t m)
    {   m_ = m; }

public:
    /// destructor virtual so user derived class destructor gets called
    virtual ~cppad_ipopt_fg_info(void)
    { }
    /// number_functions; i.e. K (simple representation uses 1)
    virtual size_t number_functions(void)
    {   return 1; }
    /// function that evaluates the users representation for f(x) and
    /// and g(x) is pure virtual so user must define it in derived class
    virtual ADVector eval_r(size_t k, const ADVector& u) = 0;
    /// should the function r_k (u) be retaped when ever the arguemnt
    /// u changes (default is true which is safe but slow)
    virtual bool retape(size_t k)
    {   return true; }
    /// domain_size q[k] for r_k (u) (simple representation uses n)
    virtual size_t domain_size(size_t k)
    {   return n_; }
    /// range_size p[k] for r_k (u) (simple representation uses m+1)
    virtual size_t range_size(size_t k)
    {   return m_ + 1; }
    /// number_terms that use r_k (u) (simple represenation uses 1)
    virtual size_t number_terms(size_t k)
    {   return 1; }
    /// return the index vectors I_{k,ell} and J_{k,ell}
    /// (simple representation uses I[i] = i and J[j] = j)
    virtual void index(size_t k, size_t ell, SizeVector& I, SizeVector& J)
    {   assert( I.size() >= m_ + 1 );
        assert( J.size() >= n_ );
        for(size_t i = 0; i <= m_; i++)
            I[i] = i;
        for(size_t j = 0; j < n_; j++)
            J[j] = j;
    }
};

/*!
Class that contains information about the problem solution

\section Nonlinear_Programming_Problem Nonlinear Programming Problem
We are give smooth functions
\f$ f : {\bf R}^n \rightarrow {\bf R} \f$
and
\f$ g : {\bf R}^n \rightarrow {\bf R}^m \f$
and wish to solve the problem
\f[
\begin{array}{rcl}
{\rm minimize} & f(x) & {\rm w.r.t.} \; x \in {\bf R}^n
\\
{\rm subject \; to} & g^l \leq g(x) \leq g^u
\\
& x^l \leq x \leq x^u
\end{array}
\f]


\section Users_Representation Users Representation
The functions
\f$ f : {\bf R}^n \rightarrow {\bf R} \f$ and
\f$ g : {\bf R}^n \rightarrow {\bf R}^m \f$ are defined by
\f[
\left( \begin{array}{c} f(x) \\ g(x) \end{array} \right)
=
\sum_{k=0}^{K-1} \; \sum_{\ell=0}^{L(k) - 1}
[ (m+1) \otimes I_{k,\ell} ] \; \circ
     \; r_k \; \circ \; [ J_{k,\ell} \otimes n ] \; (x)
\f]
where for \f$ k = 0 , \ldots , K-1\f$,
\f$ r_k : {\bf R}^{q(k)} \rightarrow {\bf R}^{p(k)} \f$.

\section Deprecated_Evaluation_Methods Evaluation Methods
The set of evaluation methods for this class is
\verbatim
    { eval_f, eval_grad_f, eval_g, eval_jac_g, eval_h }
\endverbatim
Note that the bool return flag for the evaluations methods
does not appear in the Ipopt documentation.
Looking at the code, it seems to be a flag telling Ipopt to abort
when the flag is false.

*/
class cppad_ipopt_solution
{
public:
    /// possible values for he solution status
    enum solution_status {
        not_defined,
        success,
        maxiter_exceeded,
        stop_at_tiny_step,
        stop_at_acceptable_point,
        local_infeasibility,
        user_requested_stop,
        feasible_point_found,
        diverging_iterates,
        restoration_failure,
        error_in_step_computation,
        invalid_number_detected,
        too_few_degrees_of_freedom,
        internal_error,
        unknown
    }  status;
    /// the approximation solution
    NumberVector      x;
    /// Lagrange multipliers corresponding to lower bounds on x
    NumberVector      z_l;
    /// Lagrange multipliers corresponding to upper bounds on x
    NumberVector      z_u;
    /// value of g(x)
    NumberVector      g;
    /// Lagrange multipliers correspondiing constraints on g(x)
    NumberVector      lambda;
    /// value of f(x)
    Ipopt::Number     obj_value;
    /// constructor initializes solution status as not yet defined
    cppad_ipopt_solution(void)
    {   status = not_defined; }
};

/*!
Class connects Ipopt to CppAD for derivative and sparsity pattern calculations.
*/
class cppad_ipopt_nlp : public Ipopt::TNLP
{
private:
    /// A Scalar value used by Ipopt
    typedef Ipopt::Number                         Number;
    /// An index value used by Ipopt
    typedef Ipopt::Index                          Index;
    /// Indexing style used in Ipopt sparsity structure
    typedef Ipopt::TNLP::IndexStyleEnum           IndexStyleEnum;
    /// A simple vector of boolean values
    typedef CppAD::vectorBool                     BoolVector;
    /// A simple vector of AD function objects
    typedef CppAD::vector< CppAD::ADFun<Number> > ADFunVector;
    /// A simple vector of simple vectors of boolean values
    typedef CppAD::vector<BoolVector>             BoolVectorVector;
    /// A mapping that is dense in i, sparse in j, and maps (i, j)
    /// to the corresponding sparsity index in Ipopt.
    typedef CppAD::vector< std::map<size_t,size_t> > IndexMap;

    // ------------------------------------------------------------------
    // Values directly passed in to constuctor
    // ------------------------------------------------------------------
    /// dimension of the domain space for f(x) and g(x)
    /// (passed to ctor)
    const size_t                    n_;
    /// dimension of the range space for g(x)
    /// (passed to ctor)
    const size_t                    m_;
    /// dimension of the range space for g(x)
    /// (passed to ctor)
    const NumberVector              x_i_;
    /// lower limit for x
    /// (size n_), (passed to ctor)
    const NumberVector              x_l_;
    /// upper limit for x
    /// (size n_) (passed to ctor)
    const NumberVector              x_u_;
    /// lower limit for g(x)
    /// (size m_) (passed to ctor)
    const NumberVector              g_l_;
    /// upper limit for g(x)
    /// (size m_) (passed to ctor)
    const NumberVector              g_u_;
    /// pointer to base class version of derived class object used to get
    /// information about the user's representation for f(x) and g(x)
    /// (passed to ctor)
    cppad_ipopt_fg_info* const      fg_info_;
    /// pointer to object where final results are stored
    /// (passed to ctor)
    cppad_ipopt_solution* const     solution_;
    /// plus infinity as a value of type Number
    const Number                    infinity_;

    // ------------------------------------------------------------------
    // Effectively const values determined during constructor using calls
    // to fg_info:
    // ------------------------------------------------------------------
    /// The value of \f$ K \f$ in the representation.
    /// (effectively const)
    size_t                 K_;
    /// Does operation sequence for \f$ r_k (u) \f$ depend on \f$ u \f$.
    /// (size K_) (effectively const)
    BoolVector             retape_;
    /// <tt>q_[k]</tt> is the domain space dimension for \f$ r_k (u) \f$
    /// (size K_) (effectively const)
    SizeVector             q_;
    /// <tt>p_[k]</tt> is the range space dimension for \f$ r_k (u) \f$
    /// (size K_) (effectively const)
    SizeVector             p_;
    /// <tt>L_[k]</tt> is number of times \f$ r_k (u) \f$ appears in
    /// the representation summation
    /// (size K_) (effectively const)
    SizeVector             L_;
    // -------------------------------------------------------------------
    // Other effectively const values determined by the constructor:
    // -------------------------------------------------------------------
    /*!
    CppAD sparsity patterns for \f$ \{ r_k^{(1)} (u) \} \f$ (set by ctor).

    For <tt>k = 0 , ... , K_-1, pattern_jac_r_[k]</tt>
    is a CppAD sparsity pattern for the Jacobian of \f$ r_k (u) \f$
    and as such it has size <tt>p_[k]*q_[k]</tt>.
    (effectively const)
    */
    BoolVectorVector                 pattern_jac_r_;

    /*!
    CppAD sparsity patterns for \f$ \{ r_k^{(2)} (u) \} \f$ (set by ctor).

    For <tt>k = 0 , ... , K_-1, pattern_jac_r_[k]</tt>
    is a CppAD sparsity pattern for the Hessian of
    \f[
        R(u) = \sum_{i=0}^{p[k]-1}  r_k (u)_i
    \f]
    and as such it has size <tt>q_[k]*q_[k]</tt>.
    (effectively const)
    */
    BoolVectorVector                 pattern_hes_r_;

    /// number non-zero is Ipopt sparsity structor for Jacobian of g(x)
    /// (effectively const)
    size_t                           nnz_jac_g_;
    /// row indices in Ipopt sparsity structor for Jacobian of g(x)
    /// (effectively const)
    SizeVector                       iRow_jac_g_;
    /// column indices in Ipopt sparsity structor for Jacobian of g(x)
    /// (effectively const)
    SizeVector                       jCol_jac_g_;

    /// number non-zero is Ipopt sparsity structor for Hessian of Lagragian
    /// (effectively const)
    size_t                           nnz_h_lag_;
    /// row indices in Ipopt sparsity structor for Hessian of Lagragian
    /// (effectively const)
    SizeVector                       iRow_h_lag_;
    /// column indices in Ipopt sparsity structor for Hessian of Lagragian
    /// (effectively const)
    SizeVector                       jCol_h_lag_;

    /*!
    Mapping from (i, j) in Jacobian of g(x) to Ipopt sparsity structure

    For <tt>i = 0 , ... , m_-1, index_jac_g_[i]</tt>
    is a standard map from column index values j to the corresponding
    index in the Ipopt sparsity structure for the Jacobian of g(x).
    */
    IndexMap                         index_jac_g_;

    /*!
    Mapping from (i, j) in Hessian of fg(x) to Ipopt sparsity structure

    For <tt>i = 0 , ... , n_-1, index_hes_fg_[i]</tt>
    is a standard map from column index values j to the corresponding
    index in the Ipopt sparsity structure for the Hessian of the Lagragian.
    */
    IndexMap                         index_hes_fg_;
    // -----------------------------------------------------------------
    // Values that are changed by routine other than the constructor:
    // -----------------------------------------------------------------

    /// For <tt>k = 0 , ... , K_-1, r_fun_[k]</tt>
    /// is a the CppAD function object corresponding to \f$ r_k (u) \f$.
    ADFunVector                      r_fun_;
    /*!
    Is r_fun[k] OK for current x.

    For <tt>k = 0 , ... , K_-1, tape_ok_[k]</tt>
    is true if current operations sequence in <tt>r_fun_[k]</tt>
    OK for this value of \f$ x \f$.
    Note that \f$ u = [ J_{k,\ell} \otimes n ] (x) \f$ may depend on the
    value of \f$ \ell \f$.
    */
    BoolVector             tape_ok_;

    /// work space of size equal maximum of <tt>q[k]</tt> w.r.t k.
    SizeVector             J_;
    /// work space of size equal maximum of <tt>p[k]</tt> w.r.t k.
    SizeVector             I_;
    // ------------------------------------------------------------
    // Private Methods
    // ------------------------------------------------------------
    /// block the default constructor from use
    cppad_ipopt_nlp(const cppad_ipopt_nlp&);
    /// blocks the assignment operator from use
    cppad_ipopt_nlp& operator=(const cppad_ipopt_nlp&);
public:
    // ----------------------------------------------------------------
    // See cppad_ipopt_nlp.cpp for doxygen documentation of these methods
    // ----------------------------------------------------------------

    /// only constructor for cppad_ipopot_nlp
    cppad_ipopt_nlp(
        size_t n                         ,
        size_t m                         ,
        const NumberVector    &x_i       ,
        const NumberVector    &x_l       ,
        const NumberVector    &x_u       ,
        const NumberVector    &g_l       ,
        const NumberVector    &g_u       ,
        cppad_ipopt_fg_info*   fg_info   ,
        cppad_ipopt_solution*  solution
    );

    // use virtual so that derived class destructor gets called.
    virtual ~cppad_ipopt_nlp();

    // return info about the nlp
    virtual bool get_nlp_info(
        Index&          n           ,
        Index&          m           ,
        Index&          nnz_jac_g   ,
        Index&          nnz_h_lag   ,
        IndexStyleEnum& index_style
    );

    // return bounds for my problem
    virtual bool get_bounds_info(
        Index           n   ,
        Number*         x_l ,
        Number*         x_u ,
        Index           m   ,
        Number*         g_l ,
        Number*         g_u
    );

    // return the starting point for the algorithm
    virtual bool get_starting_point(
        Index          n            ,
        bool           init_x       ,
        Number*        x            ,
        bool           init_z       ,
        Number*        z_L          ,
        Number*        z_U          ,
        Index          m            ,
        bool           init_lambda  ,
        Number*        lambda
    );

    // return the objective value
    virtual bool eval_f(
        Index          n           ,
        const Number*  x           ,
        bool           new_x       ,
        Number&        obj_value
    );

    // Method to return the gradient of the objective
    virtual bool eval_grad_f(
        Index          n           ,
        const Number*  x           ,
        bool           new_x       ,
        Number*        grad_f
    );

    // return the constraint residuals
    virtual bool eval_g(
        Index          n           ,
        const Number*  x           ,
        bool           new_x       ,
        Index          m           ,
        Number*        g
    );

    // Method to return:
    // 1) The structure of the jacobian (if "values" is NULL)
    // 2) The values of the jacobian (if "values" is not NULL)
    virtual bool eval_jac_g(
        Index          n           ,
        const Number*  x           ,
        bool           new_x       ,
        Index          m           ,
        Index          nele_jac    ,
        Index*         iRow        ,
        Index*         jCol        ,
        Number*        values
    );

    // Method to return:
    //  1) structure of hessian of the lagrangian (if "values" is NULL)
    //  2) values of hessian of the lagrangian (if "values" is not NULL)
    virtual bool eval_h(
        Index          n           ,
        const Number*  x           ,
        bool           new_x       ,
        Number         obj_factor  ,
        Index          m           ,
        const Number*  lambda      ,
        bool           new_lambda  ,
        Index          nele_hess   ,
        Index*         iRow        ,
        Index*         jCol        ,
        Number*        values
    );

    // called when the algorithm is completed so the TNLP can
    // store/write the solution
    virtual void finalize_solution(
        Ipopt::SolverReturn       status      ,
        Index                      n          ,
        const Number*              x          ,
        const Number*              z_L        ,
        const Number*              z_U        ,
        Index                      m          ,
        const Number*              g          ,
        const Number*              lambda     ,
        Number                     obj_value  ,
        const Ipopt::IpoptData*           ip_data    ,
        Ipopt::IpoptCalculatedQuantities* ip_cq
    );

    virtual bool intermediate_callback(
        Ipopt::AlgorithmMode              mode,
        Index                             iter,
        Number                            obj_value,
        Number                            inf_pr,
        Number                            inf_du,
        Number                            mu,
        Number                            d_norm,
        Number                            regularization_size,
        Number                            alpha_du,
        Number                            alpha_pr,
        Index                             ls_trials,
        const Ipopt::IpoptData*           ip_data,
        Ipopt::IpoptCalculatedQuantities* ip_cq
    );

};


// ---------------------------------------------------------------------------
} // end namespace cppad_ipopt
// ---------------------------------------------------------------------------

# endif
