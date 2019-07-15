﻿/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "statistics.h"

#include "TimerBase.h"

#include <iostream>

namespace Saiga
{
template <typename TimerType = ScopedTimer<float>, typename F, typename... Ts>
inline Statistics<float> measureFunction(const std::string& name, int its, F f, Ts&... args)
{
    std::vector<float> timings(its);
    for (int i = 0; i < its; ++i)
    {
        float time;
        {
            TimerType tim(time);
            f(args...);
        }
        timings[i] = time;
    }
    auto st = Statistics<float>(timings);
    std::cout << "> Measured execution time of function " << name << " in ms." << std::endl;
    std::cout << st << std::endl;
    return st;
}



template <typename TimerType = ScopedTimer<float>, typename F>
inline Statistics<float> measureObject(int its, F f)
{
    std::vector<float> timings(its);
    for (int i = 0; i < its; ++i)
    {
        float time;
        {
            TimerType tim(time);
            f();
        }
        timings[i] = time;
    };
    return Statistics<float>(timings);
}

template <typename TimerType = ScopedTimer<float>, typename F>
inline Statistics<float> measureObject(const std::string& name, int its, F f)
{
    auto st = measureObject<TimerType>(its, f);
    std::cout << "> Measured execution time of function " << name << " in ms." << std::endl;
    std::cout << st << std::endl;
    return st;
}


}  // namespace Saiga
