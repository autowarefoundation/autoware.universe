#ifndef CPPAD_CG_CPPADCG_ASSERT_INCLUDED
#define CPPAD_CG_CPPADCG_ASSERT_INCLUDED
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

#define CPPADCG_ASSERT_KNOWN(exp, msg)          \
{	if( ! ( exp ) )                             \
	CppAD::ErrorHandler::Call(                  \
		true       ,                            \
		__LINE__   ,                            \
 		__FILE__   ,                            \
		#exp       ,                            \
		msg        );                           \
}


#ifdef NDEBUG
#define CPPADCG_ASSERT_UNKNOWN(exp)      // do nothing
#else
#define CPPADCG_ASSERT_UNKNOWN(exp)             \
{	if( ! ( exp ) )                             \
	CppAD::ErrorHandler::Call(                  \
		false      ,                            \
		__LINE__   ,                            \
 		__FILE__   ,                            \
		#exp       ,                            \
		""         );                           \
}
#endif

#ifndef NDEBUG
#define CPPADCG_IF_DEBUG(arg) arg
#else
#define CPPADCG_IF_DEBUG(arg)
#endif

#endif
