/**
 * This file is part of the Eigen Recursive Matrix Extension (ERME).
 *
 * Copyright (c) 2019 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "../Cholesky.h"
#include "../Core.h"
#include "MixedSolver.h"

namespace Eigen::Recursive
{
/**
 * A spezialized solver for BundleAjustment-like problems.
 * The structure is a follows:
 *
 * | U  W |
 * | WT V |
 *
 * Where,
 * U : Diagonalmatrix
 * V : Diagonalmatrix
 * W : Sparsematrix
 *
 * This solver computes the schur complement on U and solves the reduced system with CG.
 */
template <typename UBlock, typename VBlock, typename WBlock, typename XType>
class MixedSymmetricRecursiveSolver<
    SymmetricMixedMatrix2<Eigen::DiagonalMatrix<UBlock, -1>, Eigen::DiagonalMatrix<VBlock, -1>,
                          Eigen::SparseMatrix<WBlock, Eigen::RowMajor>>,
    XType>
{
   public:
    using AType = SymmetricMixedMatrix2<Eigen::DiagonalMatrix<UBlock, -1>, Eigen::DiagonalMatrix<VBlock, -1>,
                                        Eigen::SparseMatrix<WBlock, Eigen::RowMajor>>;

    using AUType = typename AType::UType;
    using AVType = typename AType::VType;
    using AWType = typename AType::WType;

    using AWTType = typename TransposeType<AWType>::Type;

    using XUType = typename XType::UType;
    using XVType = typename XType::VType;

    using S1Type = Eigen::SparseMatrix<UBlock, Eigen::RowMajor>;
    using S2Type = Eigen::SparseMatrix<VBlock, Eigen::RowMajor>;

    using LDLT = Eigen::RecursiveSimplicialLDLT<S1Type, Eigen::Upper>;


    using InnerSolver1 = MixedSymmetricRecursiveSolver<S1Type, XUType>;
    using InnerSolver2 = MixedSymmetricRecursiveSolver<S2Type, XVType>;

    void analyzePattern(const AType& A, const LinearSolverOptions& solverOptions)
    {
        n = A.u.rows();
        m = A.v.rows();

        Vinv.resize(m);
        Y.resize(n, m);
        Sdiag.resize(n);
        ej.resize(n);
        q.resize(m);
        S1.resize(n, n);


        if (solverOptions.solverType == LinearSolverOptions::SolverType::Direct)
        {
            hasWT         = true;
            explizitSchur = true;
        }
        else
        {
            // TODO: add heurisitc here
            hasWT         = true;
            explizitSchur = false;
        }

        if (hasWT)
        {
            transposeStructureOnly(A.w, WT);
        }

        patternAnalyzed = true;
    }


    void solve(AType& A, XType& x, XType& b, const LinearSolverOptions& solverOptions = LinearSolverOptions())
    {
        // Some references for easier access
        const AUType& U  = A.u;
        const AVType& V  = A.v;
        const AWType& W  = A.w;
        XUType& da       = x.u;
        XVType& db       = x.v;
        const XUType& ea = b.u;
        const XVType& eb = b.v;


        if (!patternAnalyzed) analyzePattern(A, solverOptions);

        if (hasWT)
        {
            transposeValueOnly(A.w, WT);
        }

#if 1
        // U schur (S1)
        for (int i = 0; i < m; ++i) Vinv.diagonal()(i) = V.diagonal()(i).get().inverse();
        multSparseDiag(W, Vinv, Y);

        if (explizitSchur)
        {
            eigen_assert(hasWT);
            S1            = (Y * WT).template triangularView<Eigen::Upper>();
            S1            = -S1;
            S1.diagonal() = U.diagonal() + S1.diagonal();
        }
        else
        {
            diagInnerProductTransposed(Y, W, Sdiag);
            Sdiag.diagonal() = U.diagonal() - Sdiag.diagonal();
        }
        ej = ea + -(Y * eb);

        if (solverOptions.solverType == LinearSolverOptions::SolverType::Iterative && !explizitSchur)
        {
            // A special implicit schur solver.
            // We cannot use the recursive inner solver here.
            // (Maybe a todo for the future)
            da.setZero();
            RecursiveDiagonalPreconditioner<UBlock> P;
            Eigen::Index iters = solverOptions.maxIterativeIterations;
            double tol         = solverOptions.iterativeTolerance;

            P.resize(n);
            P.compute(Sdiag);
            XUType tmp(n);

#    if 1
            recursive_conjugate_gradient(
                [&](const XUType& v, XUType& result) {
                    // x = U * p - Y * WT * p
                    if (hasWT)
                    {
                        tmp = Y * (WT * v);
                    }
                    else
                    {
                        multSparseRowTransposedVector(W, v, q);
                        tmp = Y * q;
                    }
                    result = (U.diagonal().array() * v.array()) - tmp.array();
                },
                ej, da, P, iters, tol);
#    else
#        pragma omp parallel num_threads(14)
            {
                recursive_conjugate_gradient_OMP(
                    [&](const XUType& v, XUType& result) {
                        // x = U * p - Y * WT * p
                        sparse_mv_omp(WT, v, q);
                        sparse_mv_omp(Y, q, tmp);
#        pragma omp for
                        for (int i = 0; i < v.rows(); ++i)
                        {
                            result(i).get() = (U.diagonal()(i).get() * v(i).get()) - tmp(i).get();
                        }
                    },
                    ej, da, P, iters, tol);
            }
#    endif
        }
        else
        {
            solver1.solve(S1, da, ej, solverOptions);
        }

        // finalize
        if (hasWT)
        {
            q = WT * da;
        }
        else
        {
            multSparseRowTransposedVector(W, da, q);
        }
        q  = eb - q;
        db = multDiagVector(Vinv, q);
#else
        // V schur (S1)
        for (int i = 0; i < m; ++i) Vinv.diagonal()(i) = V.diagonal()(i).get().inverse();
        Y = multSparseDiag(W, Vinv);
#endif
    }

   private:
    int n, m;

    // ==== Solver tmps ====
    XVType q;
    AVType Vinv;
    AWType Y;
    S1Type S1;
    Eigen::DiagonalMatrix<UBlock, -1> Sdiag;
    XUType ej;

    AWTType WT;

    InnerSolver1 solver1;
    InnerSolver2 solver2;

    bool patternAnalyzed = false;
    bool hasWT           = true;
    bool explizitSchur   = true;
};


}  // namespace Eigen::Recursive
