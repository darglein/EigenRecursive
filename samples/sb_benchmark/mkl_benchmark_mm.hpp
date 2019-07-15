/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include <fstream>

#include "sparse_block_benchmark.h"
namespace Saiga
{
template <typename T, int block_size, int factor>
inline void MKL_Test<T, block_size, factor>::testResultMatrixMatrix()
{
    std::cout << "> Checking Correctness of Matrix-Matrix Mult... ";

    C = BlockMatrix(n, m);  // reset C so memory is not preallocated
    Eigen::internal::conservative_sparse_sparse_product_impl_yx(A, B, C);


#if defined(SAIGA_USE_MKL)
    mkl_sparse_spmm(SPARSE_OPERATION_NON_TRANSPOSE, mkl_A, mkl_B, &mkl_C);



    BlockMatrix mklToEigen;
    createEigenFromBlockMKL(mklToEigen, &mkl_C, block_size);

    // just compare block by block
    eigen_assert(C.nonZeros() == mklToEigen.nonZeros());
    double error = 0;
    for (auto i = 0; i < C.nonZeros(); ++i)
    {
        error += (C.valuePtr()[i].get() - mklToEigen.valuePtr()[i].get()).norm();
    }


    std::cout << "Error: " << error << std::endl;
    eigen_assert(error < 1e-10);

    mkl_sparse_destroy(mkl_C);
#else
    std::cout << "Skipped" << std::endl;
#endif
}


template <typename T, int block_size, int factor>
inline void MKL_Test<T, block_size, factor>::sparseMatrixMatrix(int smm_its)
{
    testResultMatrixMatrix();
    std::ofstream strm("eigen_mkl_mm.csv", std::ostream::app);
    std::cout << "> Running SMM Benchmark with " << smm_its << " runs. " << std::endl;
    // ============= Benchmark =============
    double flop;
    Statistics<float> stat_eigen, stat_mkl;

    // Compute stats

    long inner_mm = 0;

    // The inner block product is most likely evaluated with a simple n^3 algorithm
    // -> n^3 multiply-add instructions
    long inner_flops = block_size * block_size * block_size;

    {
        // compute the number of inner block-block multiplications
        using namespace Eigen;

        auto lhs = A;
        auto rhs = B;

        using Rhs = BlockMatrix;
        using Lhs = BlockMatrix;

        Index cols = rhs.outerSize();

        for (Index j = 0; j < cols; ++j)
        {
            for (typename Rhs::InnerIterator rhsIt(rhs, j); rhsIt; ++rhsIt)
            {
                Index k = rhsIt.index();
                for (typename Lhs::InnerIterator lhsIt(lhs, k); lhsIt; ++lhsIt)
                {
                    inner_mm++;
                }
            }
        }
    }

    // This should be a lower bound approximation of the required flops
    // There might be one additional block-add per inner_mm, but that
    // could be optimized away by the compiler, so we omit it here.
    flop = inner_mm * inner_flops;


    C.resize(n, m);
    stat_eigen = measureObject(smm_its, [&]() {  // We call the spmm kernel directly, because the multiplication
                                                 // with operator* also sorts the output columns ascending.
        // The mkl spmm doesn't sort so a comparison would have been unfair.
        C = BlockMatrix(n, m);  // reset C so memory is not preallocated
        Eigen::internal::conservative_sparse_sparse_product_impl_yx(A, B, C);
    });

#if defined(SAIGA_USE_MKL)
    stat_mkl = measureObject(smm_its, [&]() {
        mkl_sparse_spmm(SPARSE_OPERATION_NON_TRANSPOSE, mkl_A, mkl_B, &mkl_C);
        mkl_sparse_destroy(mkl_C);
    });
#endif


    // time in seconds
    double ts_eigen = stat_eigen.median / 1000.0;
    double ts_mkl   = stat_mkl.median / 1000.0;

    double gflop_eigen = flop / (ts_eigen * 1000 * 1000 * 1000);
    double gflop_mkl   = flop / (ts_mkl * 1000 * 1000 * 1000);

    std::cout << "Median Time Eigen : " << ts_eigen << " -> " << gflop_eigen << " GFlop/s" << std::endl;
    std::cout << "Median Time MKL   : " << ts_mkl << " -> " << gflop_mkl << " GFlop/s" << std::endl;

    strm << block_size << "," << n << "," << nnzr << "," << typeid(T).name() << "," << ts_eigen << "," << gflop_eigen
         << "," << ts_mkl << "," << gflop_mkl << "," << (ts_mkl / ts_eigen) << ",1" << std::endl;
}

}  // namespace Saiga
