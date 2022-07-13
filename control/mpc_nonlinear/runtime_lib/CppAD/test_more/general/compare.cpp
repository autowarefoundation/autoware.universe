/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
Check comparison operators between AD< AD<Base> > and Base, int
*/
# include <cppad/cppad.hpp>

namespace {
    template <class Type>
    bool Compare(void)
    {   bool ok = true;
        using CppAD::AD;

        Type      middle = 4;
        AD<double> three = 3;
        AD<double> four  = 4;
        AD<double> five  = 5;

        // AD<double> > Type
        ok &= ! (three  >  middle);
        ok &= ! (four   >  middle);
        ok &=   (five   >  middle);
        // Type > AD<double>
        ok &=   (middle >  three );
        ok &= ! (middle >  four  );
        ok &= ! (middle >  five  );

        // AD<double> >= Type
        ok &= ! (three  >= middle);
        ok &=   (four   >= middle);
        ok &=   (five   >= middle);
        // Type > AD<double>
        ok &=   (middle >= three );
        ok &=   (middle >= four  );
        ok &= ! (middle >= five  );

        // AD<double> < Type
        ok &=   (three  <  middle);
        ok &= ! (four   <  middle);
        ok &= ! (five   <  middle);
        // Type > AD<double>
        ok &= ! (middle <  three );
        ok &= ! (middle <  four  );
        ok &=   (middle <  five  );

        // AD<double> <= Type
        ok &=   (three  <= middle);
        ok &=   (four   <= middle);
        ok &= ! (five   <= middle);
        // Type > AD<double>
        ok &= ! (middle <= three );
        ok &=   (middle <= four  );
        ok &=   (middle <= five  );

        return ok;
    }
}
bool Compare(void)
{   bool ok = true;
    ok     &= Compare<int>();
    ok     &= Compare<double>();
    ok     &= Compare< CppAD::AD<double> >();
    return ok;
}
