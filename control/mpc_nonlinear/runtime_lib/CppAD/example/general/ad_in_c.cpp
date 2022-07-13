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
$begin ad_in_c.cpp$$
$spell
    CppAD
$$

$section Example and Test Linking CppAD to Languages Other than C++$$



$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cstdio>
# include <cppad/cppad.hpp>
# include <list>

namespace { // Begin empty namespace *****************************************

/*
void debug_print(const char *label, double d)
{   using std::printf;

    unsigned char *byte = reinterpret_cast<unsigned char *>(&d);
    size_t n_byte = sizeof(d);
    printf("%s", label);
    for(size_t i = 0; i < n_byte; i++)
        printf("%x", byte[i]);
    printf("\n");
}
*/

// type in C corresponding to an AD<double> object
typedef struct { void*  p_void; } cad;

// type in C corresponding to a an ADFun<double>
typedef struct { void* p_void; } cad_fun;

// type in C corresponding to a C AD binary operator
typedef enum { op_add, op_sub, op_mul, op_div } cad_binary_op;

// type in C corresponding to a C AD unary operator
typedef enum {
    op_abs, op_acos, op_asin, op_atan, op_cos, op_cosh,
    op_exp, op_log,  op_sin,  op_sinh, op_sqrt
} cad_unary_op;

// --------------------------------------------------------------------------
// helper code not intended for use by C code  ------------------------------
using CppAD::AD;
using CppAD::ADFun;
using CppAD::vector;
using CppAD::NearEqual;

void cad2vector(size_t n, cad* p_cad, vector< AD<double> >& v)
{   assert( n == v.size() );
    for(size_t j = 0; j < n; j++)
    {   AD<double>* p_ad =
            reinterpret_cast< AD<double>* > (p_cad[j].p_void);
        v[j] = *p_ad;
    }
}

void vector2cad(size_t n, vector< AD<double> >& v, cad* p_cad)
{   assert( n == v.size() );
    for(size_t j = 0; j < n; j++)
    {   AD<double>* p_ad =
            reinterpret_cast< AD<double>* > (p_cad[j].p_void);
        *p_ad = v[j];
    }
}

void double2vector(size_t n, double* p_dbl, vector<double>& v)
{   assert( n == v.size() );
    for(size_t j = 0; j < n; j++)
        v[j] = p_dbl[j];
}

void vector2double(size_t n, vector<double>& v, double *p_dbl)
{   assert( n == v.size() );
    for(size_t j = 0; j < n; j++)
        p_dbl[j] = v[j];
}

std::list<void*> allocated;
# ifdef NDEBUG
inline void push_allocated(void *p)
{ }
inline void pop_allocated(void *p)
{ }
# else
inline void push_allocated(void *p)
{   assert( p != 0 );
    allocated.push_front(p);
}
inline void pop_allocated(void *p)
{   std::list<void*>::iterator i;
    for(i = allocated.begin(); i != allocated.end(); ++i)
    {   if( *i == p )
        {   allocated.erase(i);
            return;
        }
    }
    assert( 0 );
}

# endif
// --------------------------------------------------------------------------
// Here is the code that links C to CppAD. You will have to add more
// functions and operators to make a complete language link.
//
extern "C"
bool cad_near_equal(double x, double y)
{   double eps = 10. * std::numeric_limits<double>::epsilon();
    return NearEqual(x, y, eps, 0.);
}

// create a C++ AD object
// value is the value that the C++ AD object will have
// p_cad->p_void: on input is 0, on output points to C++ AD object
extern "C"
void cad_new_ad(cad *p_cad, double value)
{   // make sure pointer is not currently allocated
    assert( p_cad->p_void == 0 );

    AD<double>* p_ad   = new AD<double>(value);
    p_cad->p_void      = reinterpret_cast<void*>(p_ad);

    // put in list of allocate pointers
    push_allocated( p_cad->p_void );
}

// delete a C++ AD object
// p_cad->value: not used
// p_cad->p_void: on input points to C++ AD object, on output is 0
extern "C"
void cad_del_ad(cad* p_cad)
{   // make sure that p_cad has been allocated
    pop_allocated( p_cad->p_void );

    AD<double>* p_ad   = reinterpret_cast< AD<double>* >( p_cad->p_void );
    delete p_ad;

    // special value for pointers that are not allocated
    p_cad->p_void = 0;
}

// extract the value from a C++ AD object
// extern "C"
double cad_value(cad* p_cad)
{   AD<double>* p_ad = reinterpret_cast< AD<double>* > (p_cad->p_void);
    return Value( Var2Par(*p_ad) );
}

// preform a C AD unary operation
extern "C"
void cad_unary(cad_unary_op op, cad* p_operand, cad* p_result)
{   AD<double> *operand, *result;
    result  = reinterpret_cast< AD<double>* > (p_result->p_void);
    operand = reinterpret_cast< AD<double>* > (p_operand->p_void);
    switch(op)
    {
        case op_abs:
        *result = fabs( *operand );
        break;

        case op_acos:
        *result = acos( *operand );
        break;

        case op_asin:
        *result = asin( *operand );
        break;

        case op_atan:
        *result = atan( *operand );
        break;

        case op_cos:
        *result = cos( *operand );
        break;

        case op_cosh:
        *result = cosh( *operand );
        break;

        case op_exp:
        *result = exp( *operand );
        break;

        case op_log:
        *result = log( *operand );
        break;

        case op_sin:
        *result = sin( *operand );
        break;

        case op_sinh:
        *result = sinh( *operand );
        break;

        case op_sqrt:
        *result = sqrt( *operand );
        break;

        default:
        // not a unary operator
        assert(0);
        break;

    }
    return;
}

// perform a C AD binary operation
extern "C"
void cad_binary(cad_binary_op op, cad* p_left, cad* p_right, cad* p_result)
{   AD<double> *result, *left, *right;
    result = reinterpret_cast< AD<double>* > (p_result->p_void);
    left   = reinterpret_cast< AD<double>* > (p_left->p_void);
    right  = reinterpret_cast< AD<double>* > (p_right->p_void);
    assert( result != 0 );
    assert( left != 0 );
    assert( right != 0 );

    switch(op)
    {   case op_add:
        *result         = *left + (*right);
        break;

        case op_sub:
        *result         = *left - (*right);
        break;

        case op_mul:
        *result         = *left * (*right);
        break;

        case op_div:
        *result         = *left / (*right);
        break;

        default:
        // not a binary operator
        assert(0);
    }
    return;
}

// declare the independent variables in C++
extern "C"
void cad_independent(size_t n, cad* px_cad)
{   vector< AD<double> > x(n);
    cad2vector(n, px_cad, x);
    CppAD::Independent(x);
    vector2cad(n, x, px_cad);
}

// create an ADFun object in C++
extern "C"
cad_fun cad_new_fun(size_t n, size_t m, cad* px_cad, cad* py_cad)
{   cad_fun fun;

    ADFun<double>* p_adfun = new ADFun<double>;
    vector< AD<double> > x(n);
    vector< AD<double> > y(m);
    cad2vector(n, px_cad, x);
    cad2vector(m, py_cad, y);
    p_adfun->Dependent(x, y);

    fun.p_void = reinterpret_cast<void*>( p_adfun );

    // put in list of allocate pointers
    push_allocated( fun.p_void );

    return fun;
}

// delete an AD function object in C
extern "C"
void cad_del_fun(cad_fun *fun)
{   // make sure this pointer has been allocated
    pop_allocated( fun->p_void );

    ADFun<double>* p_adfun
        = reinterpret_cast< ADFun<double>* > (fun->p_void);
    delete p_adfun;

    // special value for pointers that are not allocated
    fun->p_void = 0;
}

// evaluate the Jacobian corresponding to a function object
extern "C"
void cad_jacobian(cad_fun fun,
    size_t n, size_t m, double* px, double* pjac )
{   assert( fun.p_void != 0 );

    ADFun<double>* p_adfun =
        reinterpret_cast< ADFun<double>* >(fun.p_void);
    vector<double> x(n), jac(n * m);

    double2vector(n, px, x);
    jac = p_adfun->Jacobian(x);
    vector2double(n * m, jac, pjac);
}

// forward mode
extern "C"
void cad_forward(cad_fun fun,
    size_t order, size_t n, size_t m, double* px, double* py )
{   assert( fun.p_void != 0 );

    ADFun<double>* p_adfun =
        reinterpret_cast< ADFun<double>* >(fun.p_void);
    vector<double> x(n), y(m);

    double2vector(n, px, x);
    y = p_adfun->Forward(order, x);
    vector2double(m, y, py);
}

// check that allocated list has been completely freed
extern "C"
bool cad_allocated_empty(void)
{   return allocated.empty();
}

} // End empty namespace ****************************************************

# include <math.h> // used to check results in c code below

# define N 2       // number of independent variables in example
# define M 5       // number of dependent variables in example

// -------------------------------------------------------------------------
// Here is the C code that uses the CppAD link above
bool ad_in_c(void)
{   // This routine is intentionally coded as if it were written in C
    // as an example of how you can link C, and other languages to CppAD
    bool ok = true;

    // x vector of AD objects in C
    double value;
    size_t j, n = N;
    cad X[N];
    for(j = 0; j < n; j++)
    {   value       = (double) (j+1) / (double) n;
        X[j].p_void = 0;
        cad_new_ad(X + j, value);
    }

    // y vector of AD objects in C
    size_t i, m = M;
    cad Y[M];
    for(i = 0; i < m; i++)
    {   value       = 0.; // required, but not used
        Y[i].p_void = 0;
        cad_new_ad(Y + i, value);
    }

    // declare X as the independent variable vector
    cad_independent(n, X);

    // y[0] = x[0] + x[1]
    cad_binary(op_add, X+0, X+1, Y+0);
    ok &= cad_near_equal( cad_value(Y+0), cad_value(X+0)+cad_value(X+1) );

    // y[1] = x[0] - x[1]
    cad_binary(op_sub, X+0, X+1, Y+1);
    ok &= cad_near_equal( cad_value(Y+1), cad_value(X+0)-cad_value(X+1) );

    // y[2] = x[0] * x[1]
    cad_binary(op_mul, X+0, X+1, Y+2);
    ok &= cad_near_equal( cad_value(Y+2), cad_value(X+0)*cad_value(X+1) );

    // y[3] = x[0] * x[1]
    cad_binary(op_div, X+0, X+1, Y+3);
    ok &= cad_near_equal( cad_value(Y+3), cad_value(X+0)/cad_value(X+1) );

    // y[4] = sin(x[0]) + asin(sin(x[0]))
    cad sin_x0 = { 0 };       // initialize p_void as zero
    cad_new_ad( &sin_x0, 0.);
    cad_unary(op_sin, X+0, &sin_x0);
    ok &= cad_near_equal(cad_value(&sin_x0), sin(cad_value(X+0)) );

    cad asin_sin_x0 = { 0 };  // initialize p_void as zero
    cad_new_ad( &asin_sin_x0, 0.);
    cad_unary(op_asin, &sin_x0, &asin_sin_x0);
    ok &= cad_near_equal(
        cad_value(&asin_sin_x0),
        asin( cad_value(&sin_x0) )
    );

    cad_binary(op_add, &sin_x0, &asin_sin_x0, Y+4);
    ok &= cad_near_equal(
        cad_value(Y+4),
        cad_value(&sin_x0) + cad_value(&asin_sin_x0)
    );

    // declare y as the dependent variable vector and stop recording
    // and store function object in f
    cad_fun f = cad_new_fun(n, m, X, Y);

    // now use the function object
    double x[N], jac[N * M];
    x[0] = 1.;
    x[1] = .5;

    // compute the Jacobian
    cad_jacobian(f, n, m, x, jac);

    // check the Jacobian values
    size_t k = 0;
    // partial y[0] w.r.t. x[0]
    ok &= cad_near_equal(jac[k++], 1.);
    // partial y[0] w.r.t. x[1]
    ok &= cad_near_equal(jac[k++], 1.);
    // partial y[1] w.r.t. x[0]
    ok &= cad_near_equal(jac[k++], 1.);
    // partial y[1] w.r.t. x[1]
    ok &= cad_near_equal(jac[k++], -1.);
    // partial y[2] w.r.t. x[0]
    ok &= cad_near_equal(jac[k++], x[1]);
    // partial y[2] w.r.t. x[1]
    ok &= cad_near_equal(jac[k++], x[0]);
    // partial y[3] w.r.t. x[0]
    ok &= cad_near_equal(jac[k++], 1./x[1]);
    // partial y[3] w.r.t. x[1]
    ok &= cad_near_equal(jac[k++], -x[0]/(x[1]*x[1]));
    // partial y[4] w.r.t x[0]
    ok &= cad_near_equal(jac[k++],  cos(x[0]) + 1.);
    // partial y[4] w.r.t x[1]
    ok &= cad_near_equal(jac[k++],  0.);

    // evaluate the function f at a different x
    size_t order = 0;
    double y[M];
    x[0] = .5;
    x[1] = 1.;
    cad_forward(f, order, n, m, x, y);

    // check the function values
    ok &= cad_near_equal(y[0] , x[0] + x[1] );
    ok &= cad_near_equal(y[1] , x[0] - x[1] );
    ok &= cad_near_equal(y[2] , x[0] * x[1] );
    ok &= cad_near_equal(y[3] , x[0] / x[1] );
    ok &= cad_near_equal(y[4] , sin(x[0]) + asin(sin(x[0])) );

    // delete All C++ copies of the AD objects
    cad_del_fun( &f );
    cad_del_ad( &sin_x0 );
    cad_del_ad( &asin_sin_x0 );
    for(j = 0; j < n; j++)
        cad_del_ad(X + j);
    for(i = 0; i < m; i++)
        cad_del_ad(Y + i);

    ok     &= cad_allocated_empty();
    return ok;
}

// END C++
