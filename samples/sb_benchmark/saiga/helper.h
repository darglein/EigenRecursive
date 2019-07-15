/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include <chrono>
#include <random>
#include <string>
#include <iostream>
namespace Saiga
{
namespace Random
{
inline std::mt19937& generator()
{
    static thread_local std::mt19937 gen(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    return gen;
}

inline int uniformInt(int low, int high)
{
    std::uniform_int_distribution<int> dis(low, high);
    return dis(generator());
}
inline std::vector<int> uniqueIndices(int sampleCount, int indexSize)
{

    std::vector<bool> used(indexSize, false);
    std::vector<int> data(sampleCount);

    for (int j = 0; j < sampleCount;)
    {
        int s = uniformInt(0, indexSize - 1);
        if (!used[s])
        {
            data[j] = s;
            used[s] = true;
            j++;
        }
    }
    return data;
}}


}

