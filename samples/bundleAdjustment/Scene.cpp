/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "Scene.h"

#include <fstream>

using std::cout;
using std::endl;

void Scene::fixWorldPointReferences()
{
    for (WorldPoint& wp : worldPoints)
    {
        wp.stereoreferences.clear();
        wp.valid = false;
    }


    int iid = 0;
    for (SceneImage& i : images)
    {
        int ipid      = 0;
        i.validPoints = 0;

        for (auto& ip : i.stereoPoints)
        {
            if (ip.wp >= 0)
            {
                WorldPoint& wp = worldPoints[ip.wp];
                wp.stereoreferences.emplace_back(iid, ipid);
                wp.valid = true;
                i.validPoints++;
            }
            ipid++;
        }
        iid++;
    }
}

bool Scene::valid() const
{
    int imgid = 0;
    for (const SceneImage& i : images)
    {
        if (i.extr < 0 || i.intr < 0) return false;

        if (i.extr >= (int)extrinsics.size() || i.intr >= (int)intrinsics.size()) return false;

        for (auto& ip : i.stereoPoints)
        {
            if (!ip) continue;
            if (ip.wp >= (int)worldPoints.size()) return false;
            auto& wp = worldPoints[ip.wp];
            if (!wp.isReferencedByStereoFrame(imgid)) return false;
        }
        imgid++;
    }

    for (auto& wp : worldPoints)
    {
        if (!wp.uniqueReferences()) return false;
    }
    return true;
}


std::vector<int> Scene::validImages()
{
    std::vector<int> res;
    for (int i = 0; i < (int)images.size(); ++i)
    {
        if (images[i]) res.push_back(i);
    }
    return res;
}

std::vector<int> Scene::validPoints()
{
    std::vector<int> res;
    for (int i = 0; i < (int)worldPoints.size(); ++i)
    {
        if (worldPoints[i]) res.push_back(i);
    }
    return res;
}

void Scene::addWorldPointNoise(double stddev)
{
    for (auto& wp : worldPoints)
    {
        wp.p += stddev * Vec3::Random();
    }
}

void Scene::addImagePointNoise(double stddev)
{
    for (auto& img : images)
    {
        for (auto& mp : img.stereoPoints) mp.point += stddev * Vec2::Random();
    }
}

void Scene::addExtrinsicNoise(double stddev)
{
    for (auto& e : extrinsics)
    {
        e.se3.translation() += stddev * Vec3::Random();
    }
}

void Scene::save(const std::string& file)
{
    assert(valid());

    std::cout << "Saving scene to " << file << "." << std::endl;
    std::ofstream strm(file);
    assert(strm.is_open());
    strm.precision(20);
    strm << std::scientific;


    strm << "# Saiga Scene file." << std::endl;
    strm << "#" << std::endl;
    strm << "# <num_intrinsics> <num_extrinsics> <num_images> <num_worldPoints>" << std::endl;
    strm << "# Intrinsics" << std::endl;
    strm << "# <fx> <fy> <cx> <cy>" << std::endl;
    strm << "# Extrinsics" << std::endl;
    strm << "# constant tx ty tz rx ry rz rw" << std::endl;
    strm << "# Images" << std::endl;
    strm << "# intr extr weight num_points" << std::endl;
    strm << "# wp depth px py weight" << std::endl;
    strm << "# WorldPoints" << std::endl;
    strm << "# x y z" << std::endl;
    strm << intrinsics.size() << " " << extrinsics.size() << " " << images.size() << " " << worldPoints.size() << " "
         << bf << " " << globalScale << std::endl;
    for (auto& i : intrinsics)
    {
        strm << i.fx << " " << i.fy << " " << i.cx << " " << i.cy << std::endl;
    }
    for (auto& e : extrinsics)
    {
        strm << e.constant << " " << e.se3.params().transpose() << std::endl;
    }

    for (auto& img : images)
    {
        strm << img.intr << " " << img.extr << " " << img.imageWeight << " " << img.stereoPoints.size() << std::endl;
        for (auto& ip : img.stereoPoints)
        {
            strm << ip.wp << " " << ip.depth << " " << ip.point.transpose() << " " << ip.weight << std::endl;
        }
    }

    for (auto& wp : worldPoints)
    {
        strm << wp.p.transpose() << std::endl;
    }
}


template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline std::istream& operator>>(std::istream& is, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m)
{
    for (int i = 0; i < m.rows(); ++i)
    {
        for (int j = 0; j < m.cols(); ++j)
        {
            is >> m(i, j);
        }
    }
    return is;
}

void Scene::load(const std::string& file)
{
    std::cout << "Loading scene from " << file << "." << std::endl;


    std::ifstream strm(file);
    if (!strm.is_open())
    {
        throw std::runtime_error("Could not open file: " + file);
    }


    auto consumeComment = [&]() {
        while (true)
        {
            auto c = strm.peek();
            if (c == '#')
            {
                std::string s;
                std::getline(strm, s);
            }
            else
            {
                break;
            }
        }
    };


    consumeComment();
    int num_intrinsics, num_extrinsics, num_images, num_worldPoints;
    strm >> num_intrinsics >> num_extrinsics >> num_images >> num_worldPoints >> bf >> globalScale;
    intrinsics.resize(num_intrinsics);
    extrinsics.resize(num_extrinsics);
    images.resize(num_images);
    worldPoints.resize(num_worldPoints);
    for (auto& i : intrinsics)
    {
        strm >> i.fx >> i.fy >> i.cx >> i.cy;
        //        Vec4 test;
        //        strm >> test;
        //        i = test;
    }
    for (auto& e : extrinsics)
    {
        Eigen::Map<Sophus::Vector<double, SE3::num_parameters>> v2(e.se3.data());
        Sophus::Vector<double, SE3::num_parameters> v;
        strm >> e.constant >> v;
        v2 = v;
    }

    for (auto& img : images)
    {
        int numpoints;
        strm >> img.intr >> img.extr >> img.imageWeight >> numpoints;
        img.stereoPoints.resize(numpoints);
        for (auto& ip : img.stereoPoints)
        {
            strm >> ip.wp >> ip.depth >> ip.point >> ip.weight;
        }
    }

    for (auto& wp : worldPoints)
    {
        strm >> wp.p;
    }

    fixWorldPointReferences();
    assert(valid());
}
