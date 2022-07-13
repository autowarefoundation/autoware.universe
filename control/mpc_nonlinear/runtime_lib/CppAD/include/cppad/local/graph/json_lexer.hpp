# ifndef CPPAD_LOCAL_GRAPH_JSON_LEXER_HPP
# define CPPAD_LOCAL_GRAPH_JSON_LEXER_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

  CppAD is distributed under the terms of the
               Eclipse Public License Version 2.0.

  This Source Code may also be made available under the following
  Secondary License when the conditions for such availability set forth
  in the Eclipse Public License, Version 2.0 are satisfied:
        GNU General Public License, Version 2.0 or later.
-------------------------------------------------------------------------- */

# include <string>
# include <cppad/core/cppad_assert.hpp>

// BEGIN_NAMESPACE_CPPAD_LOCAL_GRAPH
namespace CppAD { namespace local { namespace graph {

// ===========================================================================
class json_lexer {
// ===========================================================================

/*
-------------------------------------------------------------------------------
$begin json_lexer_member_data$$
$spell
    json
    lexer
$$

$section json lexer: Private Data$$

$head graph_$$
The $cref json_ad_graph$$.

$head index_$$
is the index in the graph for the current character.
If a token is returned, this corresponds to the last character
it the token.

$head line_number_$$
line number in the graph for the current character

$head char_number_$$
character number in the graph for the current character

$head token_$$
used to return tokens.

$head function_name_$$
is the function name for this graph.
This is initialized as empty,
should be set as soon as it is parsed,
and is used for error reporting.

$head token$$
returns current value of $code token_$$.

$head line_number$$
returns current value of $code line_number_$$
(which corresponds to last character in the token).

$head char_number$$
returns current value of $code char_number_$$.
(which corresponds to last character in the token).

$head set_function_name$$
sets the value of $code function_name_$$.

$head Source Code$$
$srccode%hpp% */
private:
    const std::string& json_;
    size_t             index_;
    size_t             line_number_;
    size_t             char_number_;
    std::string        token_;
    std::string        function_name_;
public:
    const std::string& token(void)       const;
    size_t             line_number(void) const;
    size_t             char_number(void) const;
    void               set_function_name(const std::string& function_name);
/* %$$
$end
-------------------------------------------------------------------------------
$begin json_lexer_report_error$$
$spell
    json
    lexer
    CppAD
$$

$section json lexer: Report an Error$$

$head Syntax$$
$codei%
    %json_lexer%.report_error(%expected%, %found%)
%$$

$head json_lexer$$
is a $code local::graph::json_lexer$$ object.

$head expected$$
is the token that is expected.

$head found$$
is the token or text that was found.

$head Report$$
The current CppAD $cref ErrorHandler$$ is used to report
an error parsing this Json AD graph.

$head Prototype$$
$srccode%hpp% */
public:
    void report_error(const std::string& expected, const std::string& found);
/* %$$
$end
-------------------------------------------------------------------------------
$begin json_lexer_next_index$$
$spell
    json
    lexer
$$

$section json lexer: Advance Index by One$$

$head Syntax$$
$codei%
    %json_lexer%.next_index()
%$$

$head json_lexer$$
is a $code local::graph::json_lexer$$ object.

$head index_$$
The input value of $code index_$$ is increased by one.
It is an error to call this routine when the input value
of $code index_$$ is greater than or equal $code json_.size()$$.

$head line_number_$$
If the previous character, before  the call, was a new line,
$code line_number_$$ is increased by one.

$head char_number_$$
If the previous character, before the call, was a new line,
$code char_number$$ is set to one.
Otherwise, $code char_number_$$ is increased by one.

$head Prototype$$
$srccode%hpp% */
private:
    void next_index(void);
/* %$$
$end
-------------------------------------------------------------------------------
$begin json_lexer_skip_white_space$$
$spell
    json
    lexer
$$

$section json lexer: Skip White Space That Separates Tokens$$

$head Syntax$$
$codei%
    %json_lexer%.skip_white_space()
%$$

$head json_lexer$$
is a json lexer object.

$head Discussion$$
This member functions is used to increase $code index_$$ until either
a non-white space character is found or $code index_$$ is equal
to $code json_.size()$$.

$head Prototype$$
$srccode%hpp% */
private:
    void skip_white_space(void);
/* %$$
$end
-------------------------------------------------------------------------------
$begin json_lexer_constructor$$
$spell
    json
    lexer
    enum
    op
    arg
$$

$section json lexer: Constructor$$

$head Syntax$$
$codei%
    local::graph::lexer %json_lexer%(%json%)
%$$

$head json$$
The argument $icode json$$ is an $cref json_ad_graph$$
and it is assumed that $icode json$$ does not change
for as long as $icode json_lexer$$ exists.

$head Initialization$$
The current token, index, line number, and character number
are set to the first non white space character in $code json_$$.
If this is not a left brace character $code '{'$$,
the error is reported and the constructor does not return.

$head Prototype$$
$srccode%hpp% */
public:
    json_lexer(const std::string& json);
/* %$$
$end
-------------------------------------------------------------------------------
$begin json_lexer_check_next_char$$
$spell
    json
    lexer
    ch
$$

$section Get and Check Next Single Character Token$$

$head Syntax$$
$codei%
    %json_lexer%.check_next_char(%ch%)
%$$

$head index_$$
The search for the character starts
at one greater than the input value for $code index_$$ and skips white space.

$head ch$$
Is a non white space
single character token that is expected.
If this character is not found,
the error is reported and this function does not return.
In the special case where $icode ch$$ is $code '\0'$$,
any non-white space character will be accepted
(but there must be such a character).

$head token_$$
If this routine returns, $code token_$$ has size one
and contains the character that is found.


$head Prototype$$
$srccode%hpp% */
public:
    void check_next_char(char ch);
/* %$$
$end
-------------------------------------------------------------------------------
$begin json_lexer_check_next_string$$
$spell
    json
    lexer
$$

$section Get and Check Next Single Character Token$$

$head Syntax$$
$codei%
    %json_lexer%.check_next_string(%expected%)
%$$

$head index_$$
The search for the string starts
at one greater than the input value for $code index_$$ and skips white space.

$head expected$$
Is the value (not including double quotes) for the string that is expected.
If this string is not found, the error is reported
and this function does not return.
In the special case where $icode expected$$ is empty,
any string will be accepted.

$head token_$$
If this routine returns,
$icode token_$$ is the string that was found.


$head Prototype$$
$srccode%hpp% */
public:
    void check_next_string(const std::string& expected);
/* %$$
$end
-------------------------------------------------------------------------------
$begin json_lexer_next_non_neg_int$$
$spell
    json
    lexer
    neg
$$

$section Get Next Non-Negative Integer$$

$head Syntax$$
$codei%
    %json_lexer%.next_non_neg_int()
    %value% = %json_lexer%.token2size_t()
%$$

$head index_$$
The search for the non-negative integer starts
at one greater than the input value for $code index_$$ and skips white space.

$head token_$$
is set to the non-negative integer.
If the next token is not a non-negative integer,
the error is reported and this function does not return.

$head value$$
If the current token is a non-negative integer,
$icode value$$ is the corresponding value.

$head Prototype$$
$srccode%hpp% */
public:
    void next_non_neg_int(void);
    size_t token2size_t(void) const;

/* %$$
$end
-------------------------------------------------------------------------------
$begin json_lexer_next_float$$
$spell
    json
    lexer
$$

$section Get Next Floating Point Number$$

$head Syntax$$
$codei%
    %ok%    = %json_lexer%.next_float()
    %value% = %json_lexer%.token2double()
%$$

$head index_$$
The search for the floating point number starts
at one greater than the input value for $code index_$$ and skips white space.

$head token_$$
is set to the floating point number.
If the next token is not a floating point number,
the error is reported and this function does not return.

$head value$$
If the current token is a floating point number,
$icode value$$ is the corresponding value.

$head Prototype$$
$srccode%hpp% */
public:
    void next_float(void);
    double token2double(void) const;

/* %$$
$end
*/

// ==========================================================================
}; // end class lexer
// ==========================================================================


} } } // END_NAMESPACE_CPPAD_LOCAL_GRAPH


# endif
