/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

  CppAD is distributed under the terms of the
               Eclipse Public License Version 2.0.

  This Source Code may also be made available under the following
  Secondary License when the conditions for such availability set forth
  in the Eclipse Public License, Version 2.0 are satisfied:
        GNU General Public License, Version 2.0 or later.
-------------------------------------------------------------------------- */
# include <cctype>
# include <cppad/local/graph/json_lexer.hpp>
# include <cppad/local/graph/cpp_graph_op.hpp>
# include <cppad/utility/error_handler.hpp>
# include <cppad/utility/to_string.hpp>
# include <cppad/utility/thread_alloc.hpp>


// BEGIN_CPPAD_LOCAL_GRAPH_NAMESPACE
namespace CppAD { namespace local { namespace graph {

// report_error
void json_lexer::report_error(
    const std::string& expected ,
    const std::string& found    )
{   size_t pos = index_;
    size_t count_newline = 0;
    while(0 < pos && count_newline < 2 )
    {   --pos;
        count_newline += json_[pos] == '\n';
    }
    std::string recent_input = json_.substr( pos, index_ - pos + 1);

    std::string msg = "Error occurred while parsing Json AD graph";
    if( function_name_ != "" )
        msg += " for the function " + function_name_;
    msg += ".\n";
    msg += "Expected a " + expected + " token but found " + found + "\n";
    msg += "Detected at end of following input:";
    msg += recent_input + "\n";
    msg += "This end is character " + to_string(char_number_);
    msg += " in line " + to_string(line_number_) + " of the json.\n";
    msg += "See https://coin-or.github.io/CppAD/doc/json_ad_graph.htm.";
    //
    // use this source code as point of detection
    bool known       = true;
    int  line        = __LINE__;
    const char* file = __FILE__;
    const char* exp  = "false";
    //
    // CppAD error handler
    ErrorHandler::Call(known, line, file, exp, msg.c_str());
}

// next_index
void json_lexer::next_index(void)
{   CPPAD_ASSERT_UNKNOWN( index_ < json_.size() );
    if( json_[index_] == '\n' )
    {   ++line_number_;
        char_number_ = 0;
    }
    ++index_;
    ++char_number_;
}

// skip_white_space
void json_lexer::skip_white_space(void)
{  while( index_ < json_.size() && isspace( json_[index_] ) )
        next_index();
}

// constructor
json_lexer::json_lexer(const std::string& json)
:
json_(json),
index_(0),
line_number_(1),
char_number_(1),
token_(""),
function_name_("")
{
    skip_white_space();
    if( index_ < json_.size() )
        token_ = json_[index_];
    if( token_ != "{" )
    {   std::string expected = "'{'";
        std::string found    = "'";
        if( index_ < json_.size() )
            found += json_[index_];
        found += "'";
        report_error(expected, found);
    }
    return;
}


// token
const std::string& json_lexer::token(void) const
{   return token_; }

// line_number
size_t json_lexer::line_number(void) const
{   return line_number_; }

// char_number
size_t json_lexer::char_number(void) const
{   return char_number_; }

// set_function_name
void json_lexer::set_function_name(const std::string& function_name)
{   function_name_ = function_name; }

// token2size_t
size_t json_lexer::token2size_t(void) const
{   return size_t( std::atoi( token_.c_str() ) ); }

// token2double
double json_lexer::token2double(void) const
{   return std::atof( token_.c_str() ); }

// check_next_char
void json_lexer::check_next_char(char ch)
{   // advance to next character
    if( index_ < json_.size() )
        next_index();
    skip_white_space();
    //
    bool ok = false;
    if( index_ < json_.size() )
    {   token_.resize(1);
        token_[0] = json_[index_];
        ok = (token_[0] == ch) | (ch == '\0');
    }
    if( ! ok )
    {   std::string expected = "a character that is not white space";
        if( ch != '\0' )
        {   expected  = "'";
            expected += ch;
            expected += "'";
        }
        //
        std::string found = "'";
        if( index_ < json_.size() )
            found += json_[index_];;
        found += "'";
        report_error(expected, found);
    }
}

// check_next_string
void json_lexer::check_next_string(const std::string& expected)
{   // advance to next character
    bool found_first_quote = index_ < json_.size();
    if( found_first_quote )
    {   next_index();
        skip_white_space();
        found_first_quote = index_ < json_.size();
    }
    // check for "
    if( found_first_quote )
        found_first_quote = json_[index_] == '"';
    //
    // set value of token
    token_.resize(0);
    if( found_first_quote )
    {   next_index();
        while( index_ < json_.size() && json_[index_] != '"' )
        {   token_.push_back( json_[index_] );
            next_index();
        }
    }
    // check for "
    bool found_second_quote = false;
    if( found_first_quote && index_ < json_.size() )
        found_second_quote = json_[index_] == '"';
    //
    bool ok = found_first_quote & found_second_quote;
    if( ok & (expected != "" ) )
        ok = expected == token_;
    if( ! ok )
    {   std::string expected_token;
        if( expected == "" )
            expected_token = "string";
        else
        {   expected_token = '"';
            expected_token += expected;
            expected_token += '"';
        }
        //
        std::string found;
        if( ! found_first_quote )
        {   found = "'";
            if( index_ < json_.size() )
                found += json_[index_];
            found += "'";
        }
        else
        {   found += '"';
            found += token_;
            if( found_second_quote )
                found += '"';
        }
        report_error(expected_token, found);
    }
}

// next_non_neg_int
void json_lexer::next_non_neg_int(void)
{   // advance to next character
    bool ok = index_ < json_.size();
    if( ok )
    {   next_index();
        skip_white_space();
        ok = index_ < json_.size();
    }
    if( ok )
        ok = std::isdigit( json_[index_] );
    if( ! ok )
    {   std::string expected_token = "non-negative integer";
        std::string found = "'";
        if( index_ < json_.size() )
            found += json_[index_];
        found += "'";
        report_error(expected_token, found);
    }
    //
    token_.resize(0);
    while( ok )
    {   token_.push_back( json_[index_] );
        ok = index_ + 1 < json_.size();
        if( ok )
            ok = isdigit( json_[index_ + 1] );
        if( ok )
            next_index();
    }
}

// next_float
void json_lexer::next_float(void)
{   // advance to next character
    bool ok = index_ < json_.size();
    if( ok )
    {   next_index();
        skip_white_space();
        ok = index_ < json_.size();
    }
    if( ok )
    {   char ch = json_[index_];
        ok = std::isdigit(ch);
        ok |= (ch == '.') | (ch == '+') | (ch == '-');
        ok |= (ch == 'e') | (ch == 'E');
    }
    if( ! ok )
    {   std::string expected_token = "floating point number";
        std::string found = "'";
        if( index_ < json_.size() )
            found += json_[index_];
        found += "'";
        report_error(expected_token, found);
    }
    //
    token_.resize(0);
    while( ok )
    {   token_.push_back( json_[index_] );
        ok = index_ + 1 < json_.size();
        if( ok )
        {   char ch  = json_[index_ + 1];
            ok  = isdigit(ch);
            ok |= (ch == '.') | (ch == '+') | (ch == '-');
            ok |= (ch == 'e') | (ch == 'E');
        }
        if( ok )
            next_index();
    }
    return;
}

} } } // END_CPPAD_LOCAL_GRAPH_NAMESPACE
