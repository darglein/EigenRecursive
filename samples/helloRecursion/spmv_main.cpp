/**
 * This file is part of the Eigen Recursive Matrix Extension (ERME).
 *
 * Copyright (c) 2019 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
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

    std::cout << "A" << std::endl << expand(A) << std::endl << std::endl;
    std::cout << "B" << std::endl << expand(B) << std::endl << std::endl;
    std::cout << "C = A * B" << std::endl << expand(C) << std::endl << std::endl;

    auto ref   = (expand(A) * expand(B)).eval();
    auto error = (ref - expand(C)).norm();
    // Might be larger than 0, because of different order of floating point operations
    std::cout << "Error to non-recursive implementation: " << error << std::endl;
    return 0;
}
