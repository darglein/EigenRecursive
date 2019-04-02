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
template <int n, int m, int outer_n, int outer_m>
class SparseTest
{
   public:
    using LHSBlock     = Matrix<double, n, m>;
    using RHSBlock     = Matrix<double, m, n>;
    using ResultBlock1 = Matrix<double, n, n>;
    using ResultBlock2 = Matrix<double, m, m>;
    using LHSVector    = Matrix<double, n, 1>;
    using RHSVector    = Matrix<double, m, 1>;


    // Fixed sized outer matrices
    using LHS     = SparseMatrix<MatrixScalar<LHSBlock>>;
    using RHS     = SparseMatrix<MatrixScalar<RHSBlock>>;
    using RHSV    = Matrix<MatrixScalar<RHSVector>, -1, 1>;
    using LHSV    = Matrix<MatrixScalar<LHSVector>, -1, 1>;
    using Result1 = SparseMatrix<MatrixScalar<ResultBlock1>>;
    using Result2 = SparseMatrix<MatrixScalar<ResultBlock2>>;



    using ExpandedType = Matrix<double, -1, -1>;

    SparseTest()
    {
        cout << "Running recursive sparse matrix tests..." << endl;
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

        add();
        scalar();
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
        // Matrix-Matrix
        Result1 resLR = L * R;
        Result2 resRL = R * L;
        // Matrix-Vector
        LHSV resLV = L * RV;
        RHSV resRV = R * LV;

        // Complex expression
        LHSV resExpr_easy = L * (R * LV);
        RHSV resExpr_hard = R * ((L * R) * LV);

        {
            // ==== Correctness check
            ExpandedType resLR2 = L2 * R2;
            ExpandedType resRL2 = R2 * L2;
            // Matrix-Vector
            ExpandedType resLV2 = L2 * RV2;
            ExpandedType resRV2 = R2 * LV2;

            ExpandedType resExpr_easy2 = L2 * (R2 * LV2);
            ExpandedType resExpr_hard2 = R2 * ((L2 * R2) * LV2);

            cout << "Mat-Mat 1 - Error: " << (expand(resLR) - resLR2).squaredNorm() << endl;
            cout << "Mat-Mat 2 - Error: " << (expand(resRL) - resRL2).squaredNorm() << endl;
            cout << "Mat-Vec 1 - Error: " << (expand(resLV) - resLV2).squaredNorm() << endl;
            cout << "Mat-Vec 2 - Error: " << (expand(resRV) - resRV2).squaredNorm() << endl;
            cout << "Expression - Error: " << (expand(resExpr_easy) - resExpr_easy2).squaredNorm() << endl;
            cout << "Expression - Error: " << (expand(resExpr_hard) - resExpr_hard2).squaredNorm() << endl;
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
    Eigen::Recursive::SparseTest<3, 2, 4, 5> test_rect;
    Eigen::Recursive::SparseTest<3, 3, 5, 5> test_square;
    return 0;
}
