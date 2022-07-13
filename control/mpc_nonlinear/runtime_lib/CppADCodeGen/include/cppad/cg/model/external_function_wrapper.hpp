#ifndef CPPAD_CG_EXTERNAL_FUNCTION_WRAPPER_INCLUDED
#define CPPAD_CG_EXTERNAL_FUNCTION_WRAPPER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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

template<class Base>
class ExternalFunctionWrapper {
public:
    /**
     * Computes results during a forward mode sweep, the Taylor coefficients 
     * for dependent variables relative to independent variables.
     * 
     * @param libModel The model calling where this is being called from.
     * @param q Lowest order for this forward mode calculation.
     * @param p Highest order for this forward mode calculation.
     * @param tx Independent variable Taylor coefficients.
     * @param ty Dependent variable Taylor coefficients.
     * @return <code>true</code> if evaluation succeeded, <code>false</code> otherwise. 
     */
    virtual bool forward(FunctorGenericModel<Base>& libModel,
                         int q,
                         int p,
                         const Array tx[],
                         Array& ty) = 0;

    /**
     * Computes results during a reverse mode sweep, the adjoints or partial
     * derivatives of independent variables.
     * 
     * @param libModel The model calling where this is being called from.
     * @param p Order for this reverse mode calculation.
     * @param tx Independent variable Taylor coefficients.
     * @param px Independent variable partial derivatives.
     * @param py Dependent variable partial derivatives.
     * @return <code>true</code> if evaluation succeeded, <code>false</code> otherwise.
     */
    virtual bool reverse(FunctorGenericModel<Base>& libModel,
                         int p,
                         const Array tx[],
                         Array& px,
                         const Array py[]) = 0;

    inline virtual ~ExternalFunctionWrapper() {
    }
};

} // END cg namespace
} // END CppAD namespace

#endif