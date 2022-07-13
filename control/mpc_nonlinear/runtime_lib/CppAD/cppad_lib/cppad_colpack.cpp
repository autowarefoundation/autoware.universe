/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <vector>
# include <cppad/utility/vector.hpp>
# include <cppad/configure.hpp>
# include <cppad/local/define.hpp>
# include <cppad/core/cppad_assert.hpp>

# if CPPAD_HAS_COLPACK == 0
namespace CppAD { namespace local {
    void this_routine_should_never_get_called(void)
    {   CPPAD_ASSERT_UNKNOWN(false); }
} }
# else // CPPAD_HAS_COLPACK
# include <ColPack/ColPackHeaders.h>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE

/*!
\file cppad_colpack.cpp
The CppAD interface to the Colpack coloring algorithms.
*/

/*!
Determine which rows of a general sparse matrix can be computed together.

\param color
is a vector with color.size() == m.
For i = 0 , ... , m-1, color[i] is the color for the corresponding row
of the matrix. If color[i1]==color[i2], (i1, j1) is in sparsity pattern,
and (i2, j2) is in sparsity pattern, then j1 is not equal to j2.

\param m
is the number of rows in the matrix.

\param n
is the number of columns in the matrix.

\param adolc_pattern
is a vector with adolc_pattern.size() == m.
For i = 0 , ... , m-1, and for k = 1, ... ,adolc_pattern[i][0],
the entry with index (i, adolc_pattern[i][k]) is a non-zero
in the sparsity pattern for the matrix.
*/
// ----------------------------------------------------------------------
void cppad_colpack_general(
    CppAD::vector<size_t>&               color         ,
    size_t                               m             ,
    size_t                               n             ,
    const CppAD::vector<unsigned int*>&  adolc_pattern )
{   size_t i, k;
    CPPAD_ASSERT_UNKNOWN( adolc_pattern.size() == m );
    CPPAD_ASSERT_UNKNOWN( color.size() == m );

    // Use adolc sparsity pattern to create corresponding bipartite graph
    ColPack::BipartiteGraphPartialColoringInterface graph(
            SRC_MEM_ADOLC,
            adolc_pattern.data(),
            m,
            n
    );

    // row ordered Partial-Distance-Two-Coloring of the bipartite graph
    graph.PartialDistanceTwoColoring(
        "SMALLEST_LAST", "ROW_PARTIAL_DISTANCE_TWO"
    );

    // ----------------------------------------------------------------------
    // If we had access to BipartiteGraphPartialColoring::m_vi_LeftVertexColors
    // we could access the coloring and not need to go through seed matrix; see
    // BipartiteGraphPartialColoring::GetLeftSeedMatrix_unmanaged in colpack
    // and cppad_colpack_symmetric below.
    // ----------------------------------------------------------------------

    // Use coloring information to create seed matrix
    int n_seed_row;
    int n_seed_col;
    double** seed_matrix = graph.GetSeedMatrix(&n_seed_row, &n_seed_col);
    CPPAD_ASSERT_UNKNOWN( size_t(n_seed_col) == m );

    // now return coloring in format required by CppAD
    for(i = 0; i < m; i++)
        color[i] = m;
    for(k = 0; k < size_t(n_seed_row); k++)
    {   for(i = 0; i < m; i++)
        {   if( seed_matrix[k][i] != 0.0 )
            {   // check that entries in the seed matrix are zero or one
                CPPAD_ASSERT_UNKNOWN( seed_matrix[k][i] == 1.0 );
                // check that no row appears twice in the coloring
                CPPAD_ASSERT_UNKNOWN( color[i] == m );
                // only need include rows with non-zero entries
                if( adolc_pattern[i][0] != 0 )
                {   // set color for this row
                    color[i] = k;
                }
            }
        }
    }
# ifndef NDEBUG
    // check non-zero versus color for each row
    for(i = 0; i < m; i++)
    {
        // if there is a color for row i, check that it has non-zero entries
        if(color[i] < m )
            CPPAD_ASSERT_UNKNOWN( adolc_pattern[i][0] != 0 );

        // if there is no color for row i, check that it is empty
        if( color[i] == m )
            CPPAD_ASSERT_UNKNOWN( adolc_pattern[i][0] == 0 );
    }

    // check that no rows with the same color have non-zero entries
    // with the same column index
    CppAD::vector<bool> found(n);
    for(k = 0; k < size_t(n_seed_row); k++)
    {   // k is the color index
        // found: column already has a non-zero entries for this color
        for(size_t j = 0; j < n; j++)
            found[j] = false;
        // for each row with color k
        for(i = 0; i < m; i++) if( color[i] == k )
        {   // for each non-zero entry in this row
            for(size_t ell = 0; ell < adolc_pattern[i][0]; ell++)
            {   // column index for this entry
                size_t j = adolc_pattern[i][1 + ell];
                // check that this is the first non-zero in this column
                CPPAD_ASSERT_UNKNOWN( ! found[j] );
                // found a non-zero in this column
                found[j] = true;
            }
        }
    }
# endif
    return;
}
// ----------------------------------------------------------------------
/*!
Determine which rows of a symmetrix sparse matrix can be computed together.

\param color
is a vector with color.size() == m.
For i = 0 , ... , m-1, color[i] is the color for the corresponding row
of the matrix. We say that a sparsity pattern entry (i, j) is valid if
for all i1, such that i1 != i and color[i1]==color[i],
and all j1, such that (i1, j1) is in sparsity pattern, j1 != j.
The coloring is chosen so that for all (i, j) in the sparsity pattern;
either (i, j) or (j, i) is valid (possibly both).

\param m
is the number of rows (and columns) in the matrix.

\param adolc_pattern
is a vector with adolc_pattern.size() == m.
For i = 0 , ... , m-1, and for k = 1, ... ,adolc_pattern[i][0],
the entry with index (i, adolc_pattern[i][k]) is
in the sparsity pattern for the symmetric matrix.
*/
void cppad_colpack_symmetric(
    CppAD::vector<size_t>&               color         ,
    size_t                               m             ,
    const CppAD::vector<unsigned int*>&  adolc_pattern )
{   size_t i;
    CPPAD_ASSERT_UNKNOWN( adolc_pattern.size() == m );
    CPPAD_ASSERT_UNKNOWN( color.size() == m );

    // Use adolc sparsity pattern to create corresponding bipartite graph
    ColPack::GraphColoringInterface graph(
            SRC_MEM_ADOLC,
            adolc_pattern.data(),
            m
    );

    // Use STAR coloring because it has a direct recovery scheme; i.e.,
    // not necessary to solve equations to extract values.
    graph.Coloring("SMALLEST_LAST", "STAR");

    // pointer to Colpack coloring solution
    const std::vector<int>* vertex_colors_ptr = graph.GetVertexColorsPtr();
    CPPAD_ASSERT_UNKNOWN( vertex_colors_ptr->size() == m );

    // now return coloring in format required by CppAD
    for(i = 0; i < m; i++)
    {   if( adolc_pattern[i][0] == 0 )
        {   // no entries in row of of sparsity patter
            color[i] = m;
        }
        else
        {   // there are non-zero entries in row i
            color[i] = size_t( (*vertex_colors_ptr)[i] );
            CPPAD_ASSERT_UNKNOWN( color[i] < m );
        }
    }

# ifndef NDEBUG
    // check that every entry in the symetric matrix can be direclty recovered
    size_t i1, i2, j1, j2, k1, k2, nz1, nz2;
    for(i1 = 0; i1 < m; i1++)
    {   nz1 = size_t(adolc_pattern[i1][0]);
        for(k1 = 1; k1 <= nz1; k1++)
        {   j1 = adolc_pattern[i1][k1];
            // (i1, j1) is a non-zero in the sparsity pattern

            // check of a forward on color[i1] followed by a reverse
            // can recover entry (i1, j1)
            bool color_i1_ok = true;
            for(i2 = 0; i2 < m; i2++) if( i1 != i2 && color[i1] == color[i2] )
            {   nz2 = adolc_pattern[i2][0];
                for(k2 = 1; k2 <= nz2; k2++)
                {   j2 = adolc_pattern[i2][k2];
                    color_i1_ok &= (j1 != j2);
                }
            }

            // check of a forward on color[j1] followed by a reverse
            // can recover entry (j1, i1)
            bool color_j1_ok = true;
            for(j2 = 0; j2 < m; j2++) if( j1 != j2 && color[j1] == color[j2] )
            {   nz2 = adolc_pattern[j2][0];
                for(k2 = 1; k2 <= nz2; k2++)
                {   i2 = adolc_pattern[j2][k2];
                    color_j1_ok &= (i1 != i2);
                }
            }

            CPPAD_ASSERT_UNKNOWN( color_i1_ok || color_j1_ok );
        }
    }
# endif
    return;
}

} } // END_CPPAD_LOCAL_NAMESPACE

# endif // CPPAD_HAS_COLPACK
