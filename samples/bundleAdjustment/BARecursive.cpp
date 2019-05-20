/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "BARecursive.h"

#include "BAPosePoint.h"

#include <fstream>
#include <numeric>

template <typename _InputIterator1, typename _InputIterator2, typename _Tp>
inline _Tp exclusive_scan(_InputIterator1 __first1, _InputIterator1 __last1, _InputIterator2 __output, _Tp __init)
{
    for (; __first1 != __last1; ++__first1, (void)++__output)
    {
        auto tmp  = *__first1;  // make sure it works inplace
        *__output = __init;
        __init    = __init + tmp;
    }
    return __init;
}
void BARec::solve(Scene& scene, const OptimizationOptions& _optimizationOptions)
{
    optimizationOptions = _optimizationOptions;
    _scene              = &scene;

    double current_chi2 = 0;
    lambda              = optimizationOptions.initialLambda;

    init();
    for (auto i = 0; i < optimizationOptions.maxIterations; ++i)
    {
        double chi2 = computeQuadraticForm();
        addLambda(lambda);
        if (i == 0)
        {
            current_chi2 = chi2;
            std::cout << "Initial chi2: " << current_chi2 << std::endl;
        }
        solveLinearSystem();
        addDelta();
        double newChi2 = computeCost();
        if (newChi2 < current_chi2)
        {
            // accept
            lambda       = lambda * (1.0 / 3.0);
            v            = 2;
            current_chi2 = newChi2;
        }
        else
        {
            // discard
            lambda = lambda * v;
            v      = 2 * v;
            revertDelta();
        }
        std::cout << "Chi2 after iteration " << i << ":  " << current_chi2 << std::endl;
    }
    finalize();
}

void BARec::init()
{
    Scene& scene = *_scene;


    if (optimizationOptions.solverType == OptimizationOptions::SolverType::Direct)
    {
        explizitSchur = true;
        computeWT     = true;
    }
    else
    {
        explizitSchur = false;
        computeWT     = true;
    }



    // Check how many valid and cameras exist and construct the compact index sets
    validPoints.clear();
    validImages.clear();
    pointToValidMap.clear();
    validImages.reserve(scene.images.size());
    validPoints.reserve(scene.worldPoints.size());
    pointToValidMap.resize(scene.worldPoints.size());

    for (int i = 0; i < (int)scene.images.size(); ++i)
    {
        auto& img = scene.images[i];
        if (!img) continue;
        validImages.push_back(i);
    }

    for (int i = 0; i < (int)scene.worldPoints.size(); ++i)
    {
        auto& wp = scene.worldPoints[i];
        if (!wp) continue;
        int validId        = validPoints.size();
        pointToValidMap[i] = validId;
        validPoints.push_back(i);
    }

    n = validImages.size();
    m = validPoints.size();


    assert(n > 0 && m > 0);

    A.resize(n, m);
    //    U.resize(n);
    //    V.resize(m);

    delta_x.resize(n, m);
    b.resize(n, m);

    x_u.resize(n);
    oldx_u.resize(n);
    x_v.resize(m);
    oldx_v.resize(m);


    // Make a copy of the initial parameters
    for (int i = 0; i < (int)validImages.size(); ++i)
    {
        auto& img = scene.images[validImages[i]];
        x_u[i]    = scene.extrinsics[img.extr].se3;
    }

    for (int i = 0; i < (int)validPoints.size(); ++i)
    {
        auto& wp = scene.worldPoints[validPoints[i]];
        x_v[i]   = wp.p;
    }

    cameraPointCounts.clear();
    cameraPointCounts.resize(n, 0);
    cameraPointCountsScan.resize(n);
    pointCameraCounts.clear();
    pointCameraCounts.resize(m, 0);
    pointCameraCountsScan.resize(m);
    observations = 0;
    for (int i = 0; i < (int)validImages.size(); ++i)
    {
        auto& img = scene.images[validImages[i]];
        for (auto& ip : img.stereoPoints)
        {
            if (ip.wp == -1) continue;

            int j = pointToValidMap[ip.wp];
            cameraPointCounts[i]++;
            pointCameraCounts[j]++;
            observations++;
        }
    }

    auto test1 = exclusive_scan(cameraPointCounts.begin(), cameraPointCounts.end(), cameraPointCountsScan.begin(), 0);
    auto test2 = exclusive_scan(pointCameraCounts.begin(), pointCameraCounts.end(), pointCameraCountsScan.begin(), 0);

    assert(test1 == observations && test2 == observations);

    // preset the outer matrix structure
    //    W.resize(n, m);
    A.w.setZero();
    A.w.reserve(observations);

    for (int k = 0; k < A.w.outerSize(); ++k)
    {
        A.w.outerIndexPtr()[k] = cameraPointCountsScan[k];
    }
    A.w.outerIndexPtr()[A.w.outerSize()] = observations;
}

double BARec::computeQuadraticForm()
{
    Scene& scene = *_scene;


    using T          = BlockBAScalar;
    using KernelType = Kernel::BAPosePointMono<T>;
    KernelType::PoseJacobiType JrowPose;
    KernelType::PointJacobiType JrowPoint;
    KernelType::ResidualType res;



    b.setZero();
    A.u.setZero();
    A.v.setZero();



    double newChi2 = 0;
    {
        int k = 0;
        for (int i = 0; i < (int)validImages.size(); ++i)
        {
            int imgid = validImages[i];
            auto& img = scene.images[imgid];
            //            auto& extr   = scene.extrinsics[img.extr].se3;
            auto& extr   = x_u[i];
            auto& extr2  = scene.extrinsics[img.extr];
            auto& camera = scene.intrinsics[img.intr];

            for (auto& ip : img.stereoPoints)
            {
                if (ip.wp == -1) continue;
                if (ip.outlier)
                {
                    A.w.valuePtr()[k].get().setZero();
                    ++k;
                    continue;
                }
                BlockBAScalar w = ip.weight * img.imageWeight * scene.scale();
                int j           = pointToValidMap[ip.wp];


                //                auto& wp        = scene.worldPoints[ip.wp].p;
                auto& wp = x_v[j];

                WElem targetPosePoint;
                auto& targetPosePose   = A.u.diagonal()(i).get();
                auto& targetPointPoint = A.v.diagonal()(j).get();
                auto& targetPoseRes    = b.u(i).get();
                auto& targetPointRes   = b.v(j).get();

                {
                    using KernelType = Kernel::BAPosePointMono<T>;
                    KernelType::PoseJacobiType JrowPose;
                    KernelType::PointJacobiType JrowPoint;
                    KernelType::ResidualType res;

                    KernelType::evaluateResidualAndJacobian(camera, extr, wp, ip.point, 1, res, JrowPose, JrowPoint);
                    if (extr2.constant) JrowPose.setZero();
                    auto c = res.squaredNorm();
                    newChi2 += c;

                    targetPosePose += JrowPose.transpose() * JrowPose;
                    targetPointPoint += JrowPoint.transpose() * JrowPoint;
                    targetPosePoint = JrowPose.transpose() * JrowPoint;
                    targetPoseRes -= JrowPose.transpose() * res;
                    targetPointRes -= JrowPoint.transpose() * res;
                }

                A.w.innerIndexPtr()[k] = j;
                A.w.valuePtr()[k]      = targetPosePoint;

                ++k;
            }
        }
    }


    return newChi2;
}

void BARec::addDelta()
{
    oldx_u = x_u;
    oldx_v = x_v;

    for (int i = 0; i < n; ++i)
    {
        auto t = delta_x.u(i).get();
        x_u[i] = SE3::exp(t) * x_u[i];
    }
    for (int i = 0; i < m; ++i)
    {
        auto t = delta_x.v(i).get();
        x_v[i] += t;
    }
}

void BARec::revertDelta()
{
    x_u = oldx_u;
    x_v = oldx_v;
}
void BARec::finalize()
{
    Scene& scene = *_scene;

    for (size_t i = 0; i < validImages.size(); ++i)
    {
        auto id    = validImages[i];
        auto& extr = scene.extrinsics[id];
        if (!extr.constant) extr.se3 = x_u[i];
    }

    for (size_t i = 0; i < validPoints.size(); ++i)
    {
        Eigen::Matrix<BlockBAScalar, 3, 1> t;
        auto id = validPoints[i];
        auto& p = scene.worldPoints[id].p;
        p       = x_v[i];
    }
}

template <typename T>
void applyLMDiagonalInner(T& diag, double lambda = 1.00e-04, double min_lm_diagonal = 1e-6,
                          double max_lm_diagonal = 1e32)
{
    for (int k = 0; k < diag.RowsAtCompileTime; ++k)
    {
        auto& value = diag.diagonal()(k);
        value       = value + lambda * value;
        value       = std::clamp(value, min_lm_diagonal, max_lm_diagonal);
    }
}


/**
 * Applies the Levenberg Marquarad Diagonal update to a recursive diagonal matrix.
 *
 * U = U + clamp(diag(U) * lambda,min,max)
 */
template <typename T>
void applyLMDiagonal(Eigen::DiagonalMatrix<T, -1>& U, double lambda = 1.00e-04, double min_lm_diagonal = 1e-6,
                     double max_lm_diagonal = 1e32)
{
    for (int i = 0; i < U.rows(); ++i)
    {
        auto& diag = U.diagonal()(i).get();
        applyLMDiagonalInner(diag, lambda, min_lm_diagonal, max_lm_diagonal);
    }
}


void BARec::addLambda(double lambda)
{
    applyLMDiagonal(A.u, lambda);
    applyLMDiagonal(A.v, lambda);
}



void BARec::solveLinearSystem()
{
    using namespace Eigen::Recursive;
    LinearSolverOptions loptions;
    loptions.maxIterativeIterations = optimizationOptions.maxIterativeIterations;
    loptions.iterativeTolerance     = optimizationOptions.iterativeTolerance;

    loptions.solverType = (optimizationOptions.solverType == OptimizationOptions::SolverType::Direct)
                              ? LinearSolverOptions::SolverType::Direct
                              : LinearSolverOptions::SolverType::Iterative;
    loptions.buildExplizitSchur = explizitSchur;

    solver.solve(A, delta_x, b, loptions);
}

double BARec::computeCost()
{
    Scene& scene = *_scene;


    using T = BlockBAScalar;

    double newChi2 = 0;
    {
        for (int i = 0; i < (int)validImages.size(); ++i)
        {
            int imgid = validImages[i];
            auto& img = scene.images[imgid];
            //            auto& extr   = scene.extrinsics[img.extr].se3;
            auto& extr   = x_u[i];
            auto& camera = scene.intrinsics[img.intr];

            for (auto& ip : img.stereoPoints)
            {
                if (!ip) continue;
                BlockBAScalar w = ip.weight * img.imageWeight * scene.scale();
                int j           = pointToValidMap[ip.wp];
                auto& wp        = x_v[j];


                {
                    using KernelType = Kernel::BAPosePointMono<T>;
                    KernelType::PoseJacobiType JrowPose;
                    KernelType::PointJacobiType JrowPoint;
                    KernelType::ResidualType res;

                    res    = KernelType::evaluateResidual(camera, extr, wp, ip.point, 1);
                    auto c = res.squaredNorm();

                    newChi2 += c;
                }
            }
        }
    }
    return newChi2;
}
