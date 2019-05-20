/**
 * This file is part of the Eigen Recursive Matrix Extension (ERME).
 *
 * Copyright (c) 2019 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "BARecursive.h"
#include "EigenRecursive/All.h"
#include "Scene.h"

#include <iostream>
using std::cout;
using std::endl;
using namespace Eigen::Recursive;

int main(int argc, char* argv[])
{
    Scene scene;
    scene.load("data/tum_large.scene");
    scene.addWorldPointNoise(0.1);
    scene.addExtrinsicNoise(0.01);
    BARec ba;
    OptimizationOptions opt;
    opt.initialLambda = 1e-2;
    ba.solve(scene, opt);
    return 0;
}
