# ifndef CPPAD_IPOPT_SOLVE_RESULT_HPP
# define CPPAD_IPOPT_SOLVE_RESULT_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
namespace ipopt {
/*!
\file solve_result.hpp
Class that contains information about solve problem result
*/

/*!
Class that contains information about solve problem result

\tparam Dvector
a simple vector with elements of type double
*/
template <class Dvector>
class solve_result
{
public:
    /// possible values for the result status
    enum status_type {
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
    };

    /// possible values for solution status
    status_type status;
    /// the approximation solution
    Dvector x;
    /// Lagrange multipliers corresponding to lower bounds on x
    Dvector zl;
    /// Lagrange multipliers corresponding to upper bounds on x
    Dvector zu;
    /// value of g(x)
    Dvector g;
    /// Lagrange multipliers correspondiing constraints on g(x)
    Dvector lambda;
    /// value of f(x)
    double obj_value;
    /// constructor initializes solution status as not yet defined
    solve_result(void)
    {   status = not_defined; }
};

} // end namespace ipopt
} // END_CPPAD_NAMESPACE

# endif
