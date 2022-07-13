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
$begin stack_machine.cpp$$
$spell
$$

$section Example Differentiating a Stack Machine Interpreter$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cstring>
# include <cstddef>
# include <cstdlib>
# include <cctype>
# include <cassert>
# include <stack>

# include <cppad/cppad.hpp>

namespace {
// Begin empty namespace ------------------------------------------------

bool is_number( const std::string &s )
{   char ch = s[0];
    bool number = (std::strchr("0123456789.", ch) != 0);
    return number;
}
bool is_binary( const std::string &s )
{   char ch = s[0];
    bool binary = (strchr("+-*/.", ch) != 0);
    return binary;
}
bool is_variable( const std::string &s )
{   char ch = s[0];
    bool variable = ('a' <= ch) & (ch <= 'z');
    return variable;
}

void StackMachine(
    std::stack< std::string >          &token_stack  ,
    CppAD::vector< CppAD::AD<double> > &variable     )
{   using std::string;
    using std::stack;

    using CppAD::AD;

    stack< AD<double> > value_stack;
    string              token;
    AD<double>          value_one;
    AD<double>          value_two;

    while( ! token_stack.empty() )
    {   string s = token_stack.top();
        token_stack.pop();

        if( is_number(s) )
        {   value_one = std::atof( s.c_str() );
            value_stack.push( value_one );
        }
        else if( is_variable(s) )
        {   value_one = variable[ size_t(s[0]) - size_t('a') ];
            value_stack.push( value_one );
        }
        else if( is_binary(s) )
        {   assert( value_stack.size() >= 2 );
            value_one = value_stack.top();
            value_stack.pop();
            value_two = value_stack.top();
            value_stack.pop();

            switch( s[0] )
            {
                case '+':
                value_stack.push(value_one + value_two);
                break;

                case '-':
                value_stack.push(value_one - value_two);
                break;

                case '*':
                value_stack.push(value_one * value_two);
                break;

                case '/':
                value_stack.push(value_one / value_two);
                break;

                default:
                assert(0);
            }
        }
        else if( s[0] == '=' )
        {   assert( value_stack.size() >= 1 );
            assert( token_stack.size() >= 1 );
            //
            s = token_stack.top();
            token_stack.pop();
            //
            assert( is_variable( s ) );
            value_one = value_stack.top();
            value_stack.pop();
            //
            variable[ size_t(s[0]) - size_t('a') ] = value_one;
        }
        else assert(0);
    }
    return;
}

// End empty namespace -------------------------------------------------------
}

bool StackMachine(void)
{   bool ok = true;

    using std::string;
    using std::stack;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    using CppAD::vector;

    // The users program in that stack machine language
    const char *program[] = {
        "1.0", "a", "+", "=", "b",  // b = a + 1
        "2.0", "b", "*", "=", "c",  // c = b * 2
        "3.0", "c", "-", "=", "d",  // d = c - 3
        "4.0", "d", "/", "=", "e"   // e = d / 4
    };
    size_t n_program = sizeof( program ) / sizeof( program[0] );

    // put the program in the token stack
    stack< string > token_stack;
    size_t i = n_program;
    while(i--)
        token_stack.push( program[i] );

    // domain space vector
    size_t n = 1;
    vector< AD<double> > X(n);
    X[0] = 0.;

    // declare independent variables and start tape recording
    CppAD::Independent(X);

    // x[0] corresponds to a in the stack machine
    vector< AD<double> > variable(26);
    variable[0] = X[0];

    // calculate the resutls of the program
    StackMachine( token_stack , variable);

    // range space vector
    size_t m = 4;
    vector< AD<double> > Y(m);
    Y[0] = variable[1];   // b = a + 1
    Y[1] = variable[2];   // c = (a + 1) * 2
    Y[2] = variable[3];   // d = (a + 1) * 2 - 3
    Y[3] = variable[4];   // e = ( (a + 1) * 2 - 3 ) / 4

    // create f : X -> Y and stop tape recording
    CppAD::ADFun<double> f(X, Y);

    // use forward mode to evaluate function at different argument value
    size_t p = 0;
    vector<double> x(n);
    vector<double> y(m);
    x[0] = 1.;
    y    = f.Forward(p, x);

    // check function values
    ok &= (y[0] == x[0] + 1.);
    ok &= (y[1] == (x[0] + 1.) * 2.);
    ok &= (y[2] == (x[0] + 1.) * 2. - 3.);
    ok &= (y[3] == ( (x[0] + 1.) * 2. - 3.) / 4.);

    // Use forward mode (because x is shorter than y) to calculate Jacobian
    p = 1;
    vector<double> dx(n);
    vector<double> dy(m);
    dx[0] = 1.;
    dy    = f.Forward(p, dx);
    ok   &= NearEqual(dy[0], 1., eps99, eps99);
    ok   &= NearEqual(dy[1], 2., eps99, eps99);
    ok   &= NearEqual(dy[2], 2., eps99, eps99);
    ok   &= NearEqual(dy[3], .5, eps99, eps99);

    // Use Jacobian routine (which automatically decides which mode to use)
    dy = f.Jacobian(x);
    ok   &= NearEqual(dy[0], 1., eps99, eps99);
    ok   &= NearEqual(dy[1], 2., eps99, eps99);
    ok   &= NearEqual(dy[2], 2., eps99, eps99);
    ok   &= NearEqual(dy[3], .5, eps99, eps99);

    return ok;
}
// END C++
