/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "sophus/se3.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

template <typename T>
using AlignedVector = std::vector<T>;


using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using SE3  = Sophus::SE3d;

template <typename T>
struct Intrinsics4Base
{
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Vec2 = Eigen::Matrix<T, 2, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    T fx, fy;
    T cx, cy;

    Intrinsics4Base() {}
    Intrinsics4Base(T fx, T fy, T cx, T cy) : fx(fx), fy(fy), cx(cx), cy(cy) {}
    Intrinsics4Base(const Vec4& v) : fx(v(0)), fy(v(1)), cx(v(2)), cy(v(3)) {}

    Vec2 project(const Vec3& X) const
    {
        auto invz = T(1) / X(2);
        auto x    = X(0) * invz;
        auto y    = X(1) * invz;
        return {fx * x + cx, fy * y + cy};
    }
};

using Intrinsics4 = Intrinsics4Base<double>;

struct Extrinsics
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Sophus::SE3d se3;
    bool constant = false;

    Eigen::Vector3d apply(const Eigen::Vector3d& X) { return se3 * X; }
};

struct WorldPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d p;
    bool valid = false;

    // Pair < ImageID, ImagePointID >
    std::vector<std::pair<int, int> > stereoreferences;


    bool uniqueReferences() const
    {
        // check if all references are unique
        auto cpy = stereoreferences;
        std::sort(cpy.begin(), cpy.end());
        auto it = std::unique(cpy.begin(), cpy.end());
        return it == cpy.end();
    }

    bool isReferencedByStereoFrame(int i) const
    {
        for (auto p : stereoreferences)
            if (p.first == i) return true;
        return false;
    }



    void removeStereoReference(int img, int ip)
    {
        assert(isReferencedByStereoFrame(img));
        for (auto& p : stereoreferences)
        {
            if (p.first == img && p.second == ip)
            {
                p = stereoreferences.back();
                break;
            }
        }
        stereoreferences.resize(stereoreferences.size() - 1);
        //        SAIGA_ASSERT(!isReferencedByStereoFrame(img));
    }

    // the valid flag is set and this point is referenced by at least one image
    bool isValid() const { return valid && (!stereoreferences.empty()); }

    explicit operator bool() const { return isValid(); }
};

struct StereoImagePoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int wp = -1;

    double depth = 0;
    Eigen::Vector2d point;
    float weight = 1;

    bool outlier = false;

    // === computed by reprojection
    double repDepth = 0;
    Eigen::Vector2d repPoint;

    explicit operator bool() const { return wp != -1 && !outlier; }
};


struct SceneImage
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int intr = -1;
    int extr = -1;
    std::vector<StereoImagePoint> stereoPoints;
    float imageWeight = 1;
    int validPoints   = 0;
    explicit operator bool() const { return valid(); }
    bool valid() const { return validPoints > 0; }
};


class Scene
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AlignedVector<Intrinsics4> intrinsics;
    AlignedVector<Extrinsics> extrinsics;
    AlignedVector<WorldPoint> worldPoints;
    AlignedVector<SceneImage> images;

    // to scale towards [-1,1] range for floating point precision
    double globalScale = 1;

    double bf = 1;


    double residualNorm2(const SceneImage& img, const StereoImagePoint& ip);
    Vec3 residual3(const SceneImage& img, const StereoImagePoint& ip);
    Vec2 residual2(const SceneImage& img, const StereoImagePoint& ip);
    double depth(const SceneImage& img, const StereoImagePoint& ip);

    // Apply a rigid transformation to the complete scene
    void transformScene(const SE3& transform);
    void rescale(double s = 1);

    // Move median to (0,0,0) and set average depth to sqrt(2)
    void normalize();

    void fixWorldPointReferences();

    bool valid() const;
    explicit operator bool() const { return valid(); }

    double chi2();
    double rms();
    double rmsDense();

    /**
     * Compute the non-zero density of the schur complement S.
     * This call is pretty expensive.
     */
    double getSchurDensity();
    double scale() { return globalScale; }

    // add 0-mean gaussian noise to the world points
    void addWorldPointNoise(double stddev);
    void addImagePointNoise(double stddev);
    void addExtrinsicNoise(double stddev);

    // projects the world points to the images and
    // sets the image point = projection
    // -> The rms will be 0 after this call
    void applyErrorToImagePoints();

    void sortByWorldPointId();

    // Computes the median point from all valid world points
    Vec3 medianWorldPoint();

    // remove all image points which project to negative depth values (behind the camera)
    void removeNegativeProjections();

    void removeOutliers(float factor);
    // removes all references to this worldpoint
    void removeWorldPoint(int id);
    void removeCamera(int id);

    // removes all worldpoints/imagepoints/images, which do not have any reference
    void compress();

    std::vector<int> validImages();
    std::vector<int> validPoints();

    // ================================= IO =================================
    // -> defined in Scene_io.cpp

    // returns true if the scene was changed by a user action
    bool imgui();
    void save(const std::string& file);
    void load(const std::string& file);
};
