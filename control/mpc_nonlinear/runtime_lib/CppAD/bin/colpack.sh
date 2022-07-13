#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ "$1" != 'forward' ] && [ "$1" != 'reverse' ]
then
    echo 'usage: ./colpack.sh option'
    echo 'where option is "forward" or "reverse"'
    exit 1
fi
if [ "$1" == 'forward' ]
then
    color_variant="COLUMN_PARTIAL_DISTANCE_TWO"
else
    color_variant="ROW_PARTIAL_DISTANCE_TWO"
fi
# ----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------
if [ ! -e 'build/colpack' ]
then
    echo_eval mkdir -p build/colpack
fi
echo 'create: build/colpack/colpack.cpp'
cat<< EOF > build/colpack/colpack.cpp
// Example using BipartiteGraphPartialColoringInterface
// to generate the seed matrix for Jacobian

#include "ColPackHeaders.h"

int main()
{   size_t i, j, k;

    using std::cout;
    using std::endl;

    //* 32x9 matrix
    size_t       i_RowCount          = 32;
    size_t       i_ColumnCount       = 9;
    size_t       i_MaxNonZerosInRows = 3;

    // JP[32][9]
    std::vector<unsigned int *> JP(i_RowCount);
    unsigned int n_data    = i_RowCount * (i_MaxNonZerosInRows + 1);
    std::vector<unsigned int> JP_memory(n_data);
    for(i = 0; i < i_RowCount; i++)
        JP[i] = JP_memory.data() + i * (i_MaxNonZerosInRows + 1);
    //
    JP[0][0] = 0;
    JP[1][0] = 1;   JP[1][1] = 0;
    JP[2][0] = 1;   JP[2][1] = 1;
    JP[3][0] = 1;   JP[3][1] = 2;
    JP[4][0] = 1;   JP[4][1] = 0;
    JP[5][0] = 3;   JP[5][1] = 0;   JP[5][2] = 1;   JP[5][3] = 3;
    JP[6][0] = 3;   JP[6][1] = 1;   JP[6][2] = 2;   JP[6][3] = 4;
    JP[7][0] = 2;   JP[7][1] = 2;   JP[7][2] = 5;
    JP[8][0] = 1;   JP[8][1] = 3;
    JP[9][0] = 3;   JP[9][1] = 3;   JP[9][2] = 4;   JP[9][3] = 6;
    JP[10][0] = 3;  JP[10][1] = 4;  JP[10][2] = 5;  JP[10][3] = 7;
    JP[11][0] = 2;  JP[11][1] = 5;  JP[11][2] = 8;
    JP[12][0] = 1;  JP[12][1] = 6;
    JP[13][0] = 2;  JP[13][1] = 6;  JP[13][2] = 7;
    JP[14][0] = 2;  JP[14][1] = 7;  JP[14][2] = 8;
    JP[15][0] = 1;  JP[15][1] = 8;
    JP[16][0] = 1;  JP[16][1] = 0;
    JP[17][0] = 2;  JP[17][1] = 0;  JP[17][2] = 1;
    JP[18][0] = 2;  JP[18][1] = 1;  JP[18][2] = 2;
    JP[19][0] = 1;  JP[19][1] = 2;
    JP[20][0] = 2;  JP[20][1] = 0;  JP[20][2] = 3;
    JP[21][0] = 3;  JP[21][1] = 1;  JP[21][2] = 3;  JP[21][3] = 4;
    JP[22][0] = 3;  JP[22][1] = 2;  JP[22][2] = 4;  JP[22][3] = 5;
    JP[23][0] = 1;  JP[23][1] = 5;
    JP[24][0] = 2;  JP[24][1] = 3;  JP[24][2] = 6;
    JP[25][0] = 3;  JP[25][1] = 4;  JP[25][2] = 6;  JP[25][3] = 7;
    JP[26][0] = 3;  JP[26][1] = 5;  JP[26][2] = 7;  JP[26][3] = 8;
    JP[27][0] = 1;  JP[27][1] = 8;
    JP[28][0] = 1;  JP[28][1] = 6;
    JP[29][0] = 1;  JP[29][1] = 7;
    JP[30][0] = 1;  JP[30][1] = 8;
    JP[31][0] = 0;
    cout << endl << "Sparsity pattern of Jacobian:" << endl;
    cout << "    ";
    for(k = 0; k < 9; k++)
        cout << setw(3) << k;
    cout << endl;
    for(i = 0; i < i_RowCount; i++)
    {   cout << setw(3) << i << ":";
        k = 0;
        for (j = 1; j <= (int) JP[i][0]; j++)
        {   while(k < JP[i][j])
            {   cout << setw(3) << 0;
                k++;
            }
            cout << setw(3) << 1;
            k++;
        }
        while(k < 9)
        {   cout << setw(3) << 0;
            k++;
        }
        cout << endl;
    }


    // Step 1: Read the sparsity pattern of the given Jacobian matrix
    // (adolc format) and create the corresponding bipartite graph
    ColPack::BipartiteGraphPartialColoringInterface g(
            SRC_MEM_ADOLC, JP.data(), i_RowCount, i_ColumnCount
    );
    g.PrintBipartiteGraph();

    // Step 2: Do Partial-Distance-Two-Coloring
    // of the bipartite graph with the specified ordering
    g.PartialDistanceTwoColoring(
        "SMALLEST_LAST", "$color_variant"
    );
    g.PrintColumnPartialColors();
    g.PrintColumnPartialColoringMetrics();

    // Step 3: From the coloring information, create and return seed matrix
    int ip1_SeedRowCount;
    int ip1_SeedColumnCount;
    double** RSeed =
        g.GetSeedMatrix(&ip1_SeedRowCount, &ip1_SeedColumnCount);
    int rows = ip1_SeedRowCount;
    int cols = ip1_SeedColumnCount;
    cout << "Seed matrix: (" << rows << "," << cols << ")" << endl;
    cout << "    ";
    for(j = 0; j < cols; j++)
        cout << setw(3) << j;
    cout << endl;
    for(i = 0; i < rows; i++)
    {   cout << setw(3) << i << ":";
        for(j = 0; j < cols; j++)
            cout << setw(3) << int(RSeed[i][j]);
        cout << endl;
    }

    return 0;
}
EOF
# ----------------------------------------------------------------------------
echo_eval cd build/colpack
echo_eval g++ colpack.cpp \
    -I$HOME/prefix/colpack/include/ColPack \
    -L$HOME/prefix/colpack/lib64 \
    -l ColPack \
    -o colpack
#
echo_eval valgrind --leak-check=yes ./colpack
# ----------------------------------------------------------------------------
echo "$0: OK"
exit 0
