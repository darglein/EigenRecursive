﻿/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


#pragma once


#include "EigenRecursive/All.h"
#include "Scene.h"

struct OptimizationOptions
{
    int maxIterations = 10;

    enum class SolverType : int
    {
        Iterative = 0,
        Direct    = 1
    };
    SolverType solverType = SolverType::Iterative;

    int maxIterativeIterations = 50;
    double iterativeTolerance  = 1e-5;

    double initialLambda = 1.00e-04;

    bool debugOutput = false;
    bool debug       = false;
};


class BARec
{
   public:
    // ============== Recusrive Matrix Types ==============
    static constexpr int blockSizeCamera = 6;
    static constexpr int blockSizePoint  = 3;
    using BlockBAScalar                  = double;

    using ADiag  = Eigen::Matrix<BlockBAScalar, blockSizeCamera, blockSizeCamera, Eigen::RowMajor>;
    using BDiag  = Eigen::Matrix<BlockBAScalar, blockSizePoint, blockSizePoint, Eigen::RowMajor>;
    using WElem  = Eigen::Matrix<BlockBAScalar, blockSizeCamera, blockSizePoint, Eigen::RowMajor>;
    using WTElem = Eigen::Matrix<BlockBAScalar, blockSizePoint, blockSizeCamera, Eigen::RowMajor>;
    using ARes   = Eigen::Matrix<BlockBAScalar, blockSizeCamera, 1>;
    using BRes   = Eigen::Matrix<BlockBAScalar, blockSizePoint, 1>;

    // Block structured diagonal matrices
    using UType = Eigen::DiagonalMatrix<Eigen::Recursive::MatrixScalar<ADiag>, -1>;
    using VType = Eigen::DiagonalMatrix<Eigen::Recursive::MatrixScalar<BDiag>, -1>;

    // Block structured vectors
    using DAType = Eigen::Matrix<Eigen::Recursive::MatrixScalar<ARes>, -1, 1>;
    using DBType = Eigen::Matrix<Eigen::Recursive::MatrixScalar<BRes>, -1, 1>;

    // Block structured sparse matrix
    using WType  = Eigen::SparseMatrix<Eigen::Recursive::MatrixScalar<WElem>, Eigen::RowMajor>;
    using WTType = Eigen::SparseMatrix<Eigen::Recursive::MatrixScalar<WTElem>, Eigen::RowMajor>;
    using SType  = Eigen::SparseMatrix<Eigen::Recursive::MatrixScalar<ADiag>, Eigen::RowMajor>;


    using BAMatrix = Eigen::Recursive::SymmetricMixedMatrix2<UType, VType, WType>;
    using BAVector = Eigen::Recursive::MixedVector2<DAType, DBType>;
    using BASolver = Eigen::Recursive::MixedSymmetricRecursiveSolver<BAMatrix, BAVector>;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BARec() {}
    ~BARec() {}
    void solve(Scene& scene, const OptimizationOptions& _optimizationOptions);
    double lambda;
    double v = 2;

   private:
    Scene* _scene;

   private:
    int n, m;

    BAMatrix A;
    BAVector x, b, delta_x;
    BASolver solver;

    AlignedVector<SE3> x_u, oldx_u;
    AlignedVector<Vec3> x_v, oldx_v;

    // ============== Structure information ==============

    int observations;
    int schurEdges;
    std::vector<std::vector<int>> schurStructure;

    // Number of seen world points for each camera + the corresponding exclusive scan and sum
    std::vector<int> cameraPointCounts, cameraPointCountsScan;
    // Number of observing cameras for each world point+ the corresponding exclusive scan and sum
    std::vector<int> pointCameraCounts, pointCameraCountsScan;


    std::vector<int> validImages;
    std::vector<int> validPoints;
    std::vector<int> pointToValidMap;



    bool explizitSchur = false;
    bool computeWT     = true;

    OptimizationOptions optimizationOptions;
    // ============== LM Functions ==============

    void init();
    double computeQuadraticForm();
    void addLambda(double lambda);
    void addDelta();
    void revertDelta();
    void solveLinearSystem();
    double computeCost();
    void finalize();
};
