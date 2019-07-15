/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


#include "sparse_block_benchmark.h"


void bench_mm();
void bench_mv();


int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    // Type of benchmark

    const int vec_mult = true;
    const int mat_mult = true;



    if (vec_mult) bench_mv();
    if (mat_mult) bench_mm();

    return 0;
}
