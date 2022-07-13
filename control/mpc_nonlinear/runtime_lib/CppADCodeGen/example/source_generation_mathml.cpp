/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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
#include <iosfwd>
#include <cppad/cg.hpp>
#include <cppad/cg/lang/mathml/mathml.hpp>

using namespace CppAD;
using namespace CppAD::cg;

int main(void) {
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    /***************************************************************************
     *                               the model
     **************************************************************************/

    // independent variable vector
    CppAD::vector<ADCG> x(2);
    x[0] = 2.;
    x[1] = 3.;
    Independent(x);

    // dependent variable vector 
    CppAD::vector<ADCG> y(1);

    // the model
    ADCG a = x[0] / 1. + x[1] * x[1];
    y[0] = a / 2;

    ADFun<CGD> fun(x, y); // the model tape

    /***************************************************************************
     *                         Generate the HMTL+MathML
     **************************************************************************/
    /**
     * start the special steps for source code generation
     */
    CodeHandler<double> handler;

    CppAD::vector<CGD> xv(x.size());
    handler.makeVariables(xv);

    CppAD::vector<CGD> yv = fun.Forward(0, xv);

    LanguageMathML<double> langMathML;
    LangMathMLDefaultVariableNameGenerator<double> nameGen;
    
    // add some additional code to select variables
    langMathML.setStyle(langMathML.getStyle() + "\n.selected{background-color: #ccc;}");

    // use block display
    langMathML.setEquationMarkup("<math display='block'>", "</math>");

    // use MathJax (and align to the left)
    langMathML.setHeadExtraMarkup("<script type='text/x-mathjax-config'>\n"
                                  //"MathJax.Hub.Config({    MMLorHTML: { prefer: { Firefox: \"MML\" } }  });\n" // use this to define a prefered browser renderer
                                  "MathJax.Hub.Config({\n"
                                  "    jax: [\"input/TeX\",\"output/HTML-CSS\"],\n"
                                  "    displayAlign: \"left\"\n"
                                  "});\n"
                                  "</script>\n"
                                  "<script type='text/javascript' src='https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML'></script>");

    langMathML.setJavascript("var selected = [];\n"
                             "\n"
                             "function clickHandler(e) {\n"
                             "    var t = e.target;\n"
                             "    for (var i = 0, len = selected.length; i < len; ++i)\n"
                             "        selected[i].classList.remove('selected');\n"
                             "    selected = [];\n"
                             "    while (t !== document) {\n"
                             "        if (t.id !== null && t.id !== \"\" && t.id.charAt(0) === 'v') {\n"
                             "            var baseId = t.id.split('_')[0];\n"
                             "            var el = document.getElementById(baseId);\n"
                             "            var n = 0;\n"
                             "            while (el !== null) {\n"
                             "                el.classList.add('selected');\n"
                             "                selected.push(el);\n"
                             "                n++;\n"
                             "                el = document.getElementById(baseId + '_' + n);\n"
                             "            }\n"
                             "            break;\n"
                             "        }\n"
                             "        t = t.parentNode;\n"
                             "    }\n"
                             "}\n"
                             "\n"
                             "document.addEventListener('DOMContentLoaded', function () {\n"
                             "    document.getElementById('algorithm').onclick = clickHandler;\n"
                             "}, false);");

    // create the HMTL file
    std::ofstream htmlFile;
    htmlFile.open("algorithm.html");

    handler.setReuseVariableIDs(false);
    handler.generateCode(htmlFile, langMathML, yv, nameGen);

    htmlFile.close();
}