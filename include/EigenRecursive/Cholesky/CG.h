﻿/**
 * This file contains (modified) code from the Eigen library.
 * Eigen License:
 *
 * Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
 * Copyright (C) 2007-2011 Benoit Jacob <jacob.benoit.1@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 *
 * ======================
 *
 * The modifications are part of the Eigen Recursive Matrix Extension (ERME).
 * ERME License:
 *
 * Copyright (c) 2019 Darius Rückert
 * Licensed under the MIT License.
 */


#pragma once
#include "../Core.h"
#include "Cholesky.h"

namespace Eigen::Recursive
{
template <typename _Scalar>
class RecursiveDiagonalPreconditioner
{
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;

   public:
    typedef typename Vector::StorageIndex StorageIndex;
    enum
    {
        ColsAtCompileTime    = Eigen::Dynamic,
        MaxColsAtCompileTime = Eigen::Dynamic
    };

    RecursiveDiagonalPreconditioner() : m_isInitialized(false) {}

    template <typename MatType>
    explicit RecursiveDiagonalPreconditioner(const MatType& mat) : m_invdiag(mat.cols())
    {
        compute(mat);
    }

    Eigen::Index rows() const { return m_invdiag.size(); }
    Eigen::Index cols() const { return m_invdiag.size(); }

    template <typename MatType>
    RecursiveDiagonalPreconditioner& analyzePattern(const MatType&)
    {
        return *this;
    }

    // Sparse Matrix Initialization
    template <typename MatType>
    RecursiveDiagonalPreconditioner& factorize(const MatType& mat)
    {
        m_invdiag.resize(mat.cols());
        for (int j = 0; j < mat.outerSize(); ++j)
        {
            typename MatType::InnerIterator it(mat, j);
            while (it && it.index() != j) ++it;
            if (it && it.index() == j)
                //          m_invdiag(j) = Scalar(1)/it.value();
                removeMatrixScalar(m_invdiag(j)) = removeMatrixScalar(inverseCholesky(it.value()));
            else
                //                m_invdiag(j) = Scalar(1);
                removeMatrixScalar(m_invdiag(j)) = removeMatrixScalar(MultiplicativeNeutral<Scalar>::get());
        }
        m_isInitialized = true;
        return *this;
    }

    template <typename T>
    RecursiveDiagonalPreconditioner& factorize(const Eigen::DiagonalMatrix<T, -1>& mat)
    {
        auto N = mat.rows();
        m_invdiag.resize(N);

        for (int j = 0; j < N; ++j)
        {
            m_invdiag(j) = inverseCholesky(mat.diagonal()(j));
        }
        m_isInitialized = true;
        return *this;
    }

    template <typename MatType>
    RecursiveDiagonalPreconditioner& compute(const MatType& mat)
    {
        return factorize(mat);
    }

    /** \internal */
    template <typename Rhs, typename Dest>
    void _solve_impl(const Rhs& b, Dest& x) const
    {
        x = m_invdiag.array() * b.array();
    }

    template <typename Rhs>
    inline const Eigen::Solve<RecursiveDiagonalPreconditioner, Rhs> solve(const Eigen::MatrixBase<Rhs>& b) const
    {
        eigen_assert(m_isInitialized && "DiagonalPreconditioner is not initialized.");
        eigen_assert(m_invdiag.size() == b.rows() &&
                     "DiagonalPreconditioner::solve(): invalid number of rows of the right hand side matrix b");
        return Eigen::Solve<RecursiveDiagonalPreconditioner, Rhs>(*this, b.derived());
    }

    Eigen::ComputationInfo info() { return Eigen::Success; }

    const auto& getDiagElement(int i) const { return m_invdiag(i); }

   protected:
    Vector m_invdiag;
    bool m_isInitialized;
};

//#define RM_CG_DEBUG_OUTPUT

/**
 * A conjugate gradient solver, which works for recursives matrices.
 * Solve:
 *          A * x = b   for x
 *
 * The matrix A is given as function (for example a lambda function).
 * This way we can implement an implicit cg solver, which does not construct the full matrix A.
 *
 * Example call:
 *
 * // Build preconditioner
 * RecursiveDiagonalPreconditioner<MatrixScalar<Block>> P;
 * Eigen::Index iters = 50;
 * Scalar tol         = 1e-50;
 * P.compute(S);
 *
 * // Solve with explicit matrix S
 * DAType tmp(n);
 * recursive_conjugate_gradient(
 *     [&](const DAType& v) {
 *         tmp = S * v;
 *         return tmp;
 *     },
 *     ej, da, P, iters, tol);
 *
 */
template <typename MultFunction, typename Rhs, typename Dest, typename Preconditioner, typename SuperScalar>
EIGEN_DONT_INLINE void recursive_conjugate_gradient(const MultFunction& applyA, const Rhs& rhs, Dest& x,
                                                    const Preconditioner& precond, Eigen::Index& iters,
                                                    SuperScalar& tol_error)
{
    // Typedefs
    using namespace Eigen;
    using std::abs;
    using std::sqrt;
    typedef SuperScalar RealScalar;
    typedef SuperScalar Scalar;
    typedef Rhs VectorType;

    // Temp Vector variables
    Index n = rhs.rows();

#ifdef RM_CG_DEBUG_OUTPUT
    cout << "Starting recursive CG" << endl;
    cout << "Iterations: " << iters << endl;
    cout << "Tolerance: " << tol_error << endl;
    cout << "N: " << n << endl;
#endif

#if 0
    // Create them locally
    VectorType z(n);
    VectorType p(n);
#else
    // Use static variables so a repeated call with the same size doesn't allocate memory
    static thread_local VectorType z;
    static thread_local VectorType p;
    static thread_local VectorType residual;
    z.resize(n);
    p.resize(n);
    residual.resize(n);
#endif



    RealScalar tol = tol_error;
    Index maxIters = iters;

    residual = rhs - applyA(x);

    RealScalar rhsNorm2 = squaredNorm(rhs);
    if (rhsNorm2 == 0)
    {
        x.setZero();
        iters     = 0;
        tol_error = 0;
        return;
    }
    RealScalar threshold     = tol * tol * rhsNorm2;
    RealScalar residualNorm2 = squaredNorm(residual);
#ifdef RM_CG_DEBUG_OUTPUT
    cout << "Initial residual: " << residualNorm2 << endl;
#endif
    if (residualNorm2 < threshold)
    {
        iters     = 0;
        tol_error = sqrt(residualNorm2 / rhsNorm2);
        return;
    }

    p = precond.solve(residual);  // initial search direction
    // the square of the absolute value of r scaled by invM
    RealScalar absNew = dot(residual, p);
#ifdef RM_CG_DEBUG_OUTPUT
    cout << "dot(r,p): " << absNew << endl;
#endif

    Index i = 0;
    while (i < maxIters)
    {
        //        cout << "CG Residual " << i << ": " << residualNorm2 << endl;
        z = applyA(p);

        // the amount we travel on dir
        Scalar alpha = absNew / dot(p, z);
        // update solution
        x += scalarMult(p, alpha);
        // update residual
        residual -= scalarMult(z, alpha);

        residualNorm2 = squaredNorm(residual);
#ifdef RM_CG_DEBUG_OUTPUT
        cout << "Iteration: " << i << " Residual: " << residualNorm2 << " Alpha: " << alpha << endl;
#endif
        if (residualNorm2 < threshold) break;

        z = precond.solve(residual);  // approximately solve for "A z = residual"
                                      //        cout << expand(p).transpose() << endl;

        RealScalar absOld = absNew;
        absNew            = dot(residual, z);  // update the absolute value of r
        RealScalar beta = absNew / absOld;  // calculate the Gram-Schmidt value used to create the new search direction
                                            //        cout << "absnew " << absNew << " beta " << beta << endl;
        p = z + scalarMult(p, beta);        // update search direction


        i++;
    }
    tol_error = sqrt(residualNorm2 / rhsNorm2);
    iters     = i;
}

}  // namespace Eigen::Recursive
