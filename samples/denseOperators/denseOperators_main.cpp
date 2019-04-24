/**
 * This file is part of the Eigen Recursive Matrix Extension (ERME).
 *
 * Copyright (c) 2019 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "EigenRecursive/All.h"



using std::cout;
using std::endl;


namespace Eigen::Recursive
{
template <int n, int m, int outer_n, int outer_m, bool fixedSize = true>
class DenseTest
{
   public:
    using LHSBlock     = Matrix<double, n, m>;
    using RHSBlock     = Matrix<double, m, n>;
    using ResultBlock1 = Matrix<double, n, n>;
    using ResultBlock2 = Matrix<double, m, m>;
    using LHSVector    = Matrix<double, n, 1>;
    using RHSVector    = Matrix<double, m, 1>;

    static constexpr int n2 = fixedSize ? outer_n : -1;
    static constexpr int m2 = fixedSize ? outer_m : -1;

#if 1
    // Fixed sized outer matrices
    using LHS     = Matrix<MatrixScalar<LHSBlock>, n2, m2>;
    using RHS     = Matrix<MatrixScalar<RHSBlock>, m2, n2>;
    using RHSV    = Matrix<MatrixScalar<RHSVector>, m2, 1>;
    using LHSV    = Matrix<MatrixScalar<LHSVector>, n2, 1>;
    using Result1 = Matrix<MatrixScalar<ResultBlock1>, n2, n2>;
    using Result2 = Matrix<MatrixScalar<ResultBlock2>, m2, m2>;
#else
    // Dynamic outer matrices
//    using LHS     = Matrix<MatrixScalar<LHSBlock>, -1, -1>;
//    using RHS     = Matrix<MatrixScalar<RHSBlock>, -1, -1>;
//    using RHSV    = Matrix<MatrixScalar<RHSVector>, -1, 1>;
//    using LHSV    = Matrix<MatrixScalar<LHSVector>, -1, 1>;
//    using Result1 = Matrix<MatrixScalar<ResultBlock1>, -1, -1>;
//    using Result2 = Matrix<MatrixScalar<ResultBlock2>, -1, -1>;
#endif


    using ExpandedType = Matrix<double, -1, -1>;

    DenseTest()
    {
        cout << "Running recursive dense matrix tests..." << endl;
        cout << "Blocks: " << n << "x" << m << endl;
        cout << "OuterSize: " << outer_n << "x" << outer_m << endl;

        L.resize(outer_n, outer_m);
        R.resize(outer_m, outer_n);
        LV.resize(outer_n, 1);
        RV.resize(outer_m, 1);

        setRandom(L);
        setRandom(R);
        setRandom(RV);
        setRandom(LV);

        L2  = expand(L);
        R2  = expand(R);
        RV2 = expand(RV);
        LV2 = expand(LV);

        //        add();
        //        scalar();
        mult();

        cout << "Done." << endl << endl;
    }

    void add() const
    {
        LHS tmp;
        tmp = -L;
        tmp = tmp + L;
        tmp = tmp - L;
        tmp = tmp + (-L);
        tmp = tmp + L - L + L;

        {
            // ==== Correctness check
            ExpandedType tmp2;
            tmp2 = -L2;
            tmp2 = tmp2 + L2;
            tmp2 = tmp2 - L2;
            tmp2 = tmp2 + (-L2);
            tmp2 = tmp2 + L2 - L2 + L2;

            double error = (expand(tmp) - tmp2).squaredNorm();
            cout << "Add - Error: " << error << endl;
        }
    }

    void scalar() const
    {
        LHS tmp;
        tmp = 0.5 * L;
        tmp = tmp * 0.5;
        tmp = 0.3 * tmp + 0.7 * tmp;

        {
            // ==== Correctness check
            ExpandedType tmp2;
            tmp2 = 0.5 * L2;
            tmp2 = tmp2 * 0.5;
            tmp2 = 0.3 * tmp2 + 0.7 * tmp2;

            double error = (expand(tmp) - tmp2).squaredNorm();
            cout << "Scalar - Error: " << error << endl;
        }
    }

    void mult() const
    {
        //        Eigen::internal::scaleAndAddTo();

        // Matrix-Vector
        LHSV resLV        = L * RV;
        RHSV resRV        = R * -LV;
        LHSV resExpr_easy = L * (R * LV);

        // Matrix-Vector
        ExpandedType resLV2        = L2 * RV2;
        ExpandedType resRV2        = R2 * -LV2;
        ExpandedType resExpr_easy2 = L2 * (R2 * LV2);

        cout << "Mat-Vec 1 - Error: " << (expand(resLV) - resLV2).squaredNorm() << endl;
        cout << "Mat-Vec 2 - Error: " << (expand(resRV) - resRV2).squaredNorm() << endl;
        cout << "Expression - Error: " << (expand(resExpr_easy) - resExpr_easy2).squaredNorm() << endl;


        // Dynamic Matrix-Matrix multiplication currently does not work
        if constexpr (fixedSize)
        {
            Result1 resLR = L * R;
            Result2 resRL = R * L;

            // Complex expression
            RHSV resExpr_hard = R * ((L * R) * LV);

            {
                // ==== Correctness check
                ExpandedType resLR2 = L2 * R2;
                ExpandedType resRL2 = R2 * L2;

                ExpandedType resExpr_hard2 = R2 * ((L2 * R2) * LV2);

                cout << "Mat-Mat 1 - Error: " << (expand(resLR) - resLR2).squaredNorm() << endl;
                cout << "Mat-Mat 2 - Error: " << (expand(resRL) - resRL2).squaredNorm() << endl;
                cout << "Expression - Error: " << (expand(resExpr_hard) - resExpr_hard2).squaredNorm() << endl;
            }
        }
    }

   private:
    LHS L;
    RHS R;
    RHSV RV;
    LHSV LV;

    ExpandedType L2;
    ExpandedType R2;
    ExpandedType RV2;
    ExpandedType LV2;
};

}  // namespace Eigen::Recursive

int main(int, char**)
{
    Eigen::Recursive::DenseTest<3, 2, 4, 5> test_rect;
    Eigen::Recursive::DenseTest<3, 3, 5, 5> test_square;

    Eigen::Recursive::DenseTest<3, 2, 4, 5, false> test_rect_dyanmic;
    Eigen::Recursive::DenseTest<3, 3, 5, 5, false> test_square_dynamic;
    return 0;
}
