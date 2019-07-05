/**
 * This file is part of the Eigen Recursive Matrix Extension (ERME).
 *
 * Copyright (c) 2019 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "EigenRecursive/All.h"
#include "EigenRecursive/Core/DenseMM.h"

#include <chrono>

using std::cout;
using std::endl;



namespace Eigen::Recursive
{
template <int L1, int L2, int L3>
class DenseBenchmark
{
   public:
    using NestedDynamic =
        Matrix<MatrixScalar<Matrix<MatrixScalar<Matrix<MatrixScalar<Matrix<double, L1, L1>>, L2, L2>>, L3, L3>>, -1,
               -1>;

    using ExpandedType = Matrix<double, -1, -1>;

    // The total size of the matrix (squared)
    int n   = 32 * 16 * 4;
    int its = 3;

    DenseBenchmark()
    {
        std::cout << "Running dense benchmark..." << std::endl;

        if (n % (L1 * L2 * L3) != 0)
        {
            throw std::runtime_error("Invalid size");
        }

        int recursiveN = n / (L1 * L2 * L3);

        nestedL.resize(recursiveN, recursiveN);
        nestedR   = nestedL;
        nestedRes = nestedL;


        setRandom(nestedL);
        setRandom(nestedR);

        nestedL2   = expand(nestedL);
        nestedR2   = expand(nestedR);
        nestedRes2 = expand(nestedRes);

        testResult();



        std::cout << "Done." << std::endl << std::endl;
    }


    void testResult() const
    {
        NestedDynamic res = nestedL * nestedR;

        ExpandedType res2 = nestedL2 * nestedR2;
        double error      = (expand(res) - res2).squaredNorm();
        std::cout << "Nested Mat-Mat 1 - Error: " << error << std::endl;

        if (error > 1e-20)
        {
            throw std::runtime_error("GEMM broken.");
        }
    }

    void benchmark()
    {
        std::vector<double> times(its);
        for (int i = 0; i < its; ++i)
        {
            times[i] = benchmarkIt();
        }
        std::sort(times.begin(), times.end());
        auto t = times[times.size() / 2];
        cout << "Recursive Median Time (" << L1 << "," << L2 << "," << L3 << "): " << t << " seconds." << endl;
    }

    void benchmarkBase()
    {
        std::vector<double> times(its);
        for (int i = 0; i < its; ++i)
        {
            times[i] = benchmarkItBase();
        }
        std::sort(times.begin(), times.end());
        auto t = times[times.size() / 2];
        cout << "Median Time: " << t << " seconds." << endl;
    }

    double benchmarkItBase()
    {
        auto startTime = std::chrono::high_resolution_clock::now();
        nestedRes2     = nestedL2 * nestedR2;
        auto endTime   = std::chrono::high_resolution_clock::now();
        auto T         = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime);
        return T.count();
    }

    double benchmarkIt()
    {
        auto startTime = std::chrono::high_resolution_clock::now();
        nestedRes      = nestedL * nestedR;
        auto endTime   = std::chrono::high_resolution_clock::now();
        auto T         = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime);
        return T.count();
    }

   private:
    NestedDynamic nestedL, nestedR, nestedRes;

    ExpandedType nestedL2, nestedR2, nestedRes2;
};

}  // namespace Eigen::Recursive

int main(int, char**)
{
    Eigen::Recursive::DenseBenchmark<2, 4, 16> test_rect;
    test_rect.benchmarkBase();
    test_rect.benchmark();
    return 0;
}
