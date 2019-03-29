# Eigen Recursive Matrix Extension

This is an extension to the [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) math library for recursive linear algebra.

Features
 * Template specializations to allow the creation and usage of recursive matrix types
 * Support for recursive sparse matrices (for example a block-sparse matrix)
 * A recursive LDLT decomposition, based on Eigen's simplicial implemenation
 * Mixed matrix types and mixed recursive solvers for structured optimization problems
 
 <img src="doc/1.png" width="425"/> <img src="doc/2.jpg" width="425"/> 
 
## Usage
	
	// See samples/helloRecursive for the full example
    #include "EigenRecursive/All.h"
    int main(int, char**)
    {
        using namespace Eigen;
		using namespace Eigen::Recursive;

        using Block          = Matrix<double, 2, 2>;
        using MatrixOfMatrix = Matrix<MatrixScalar<Block>, 2, 2>;

        MatrixOfMatrix A, B, C;

        A = RecursiveRandom<MatrixOfMatrix>::get();
        B = RecursiveRandom<MatrixOfMatrix>::get();

        C = A * B;

        std::cout << expand(C) << std::endl;
        return 0;
    }
	
## License

Copyright (c) 2019 Darius Rückert

Licensed under the MIT License.

See LICENSE file for more information.
