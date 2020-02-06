// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  TDMASolver.h
 * @brief
 *
 * TriDiagonal-Matrix Algorithm for Eigen library.
 * Solve
 * | b1 c1 0  0 ... 0 | |x1|   |d1|
 * | a2 b2 c2 0 ... 0 | |x2|   |d2|x
 * |     ...     cn-1 | |..| = |..|
 * | 0   ...    an bn | |xn|   |dn|
 *
 * @date  $Date$
 */

#include <vector>

#ifndef TDMASOLVER_H
#define TDMASOLVER_H

namespace hrp {

// Assuming a[0] = c[dim - 1] = 0
// Change array a, b, and d
std::vector<double> solveTDMA(std::vector<double>& a,
                              std::vector<double>& b,
                              const std::vector<double>& c,
                              std::vector<double>& d)
{
    const size_t dim = a.size();
    if (!(b.size() == dim && c.size() == dim && d.size() == dim)) return std::vector<double>();

    for (size_t i = 1; i < dim; ++i) {
        a[i] /= b[i - 1];
        b[i] -= -a[i] * c[i - 1];
        d[i] -= -a[i] * d[i - 1];
    }

    std::vector<double> x(dim);
    x[dim - 1] = d[dim - 1] / b[dim - 1];

    for (int i = dim - 2; i >= 0; --i) x[i] = (d[i] - c[i] * x[i + 1]) / b[i];
    return x;
}

// Assuming a[0] = c[dim - 1] = 0
std::vector<double> solveTDMAPreserve(const std::vector<double>& a,
                                      const std::vector<double>& b,
                                      const std::vector<double>& c,
                                      const std::vector<double>& d)
{
    const size_t dim = a.size();
    if (!(b.size() == dim && c.size() == dim && d.size() == dim)) return std::vector<double>();

    std::vector<double> P(dim);
    std::vector<double> Q(dim);

    P[0] = c[0] / b[0];
    Q[0] = d[0] / b[0];

    for (size_t i = 1; i < dim; ++i) {
        P[i] = c[i] / (b[i] - a[i] * P[i - 1]);
        Q[i] = (d[i] - a[i] * Q[i - 1]) / (b[i] - a[i] * P[i - 1]);
    }

    std::vector<double> x(dim);
    x[dim - 1] = Q[dim - 1];

    for (int i = dim - 2; i >= 0; --i) x[i] = Q[i] - P[i] * x[i + 1];
    return x;
}

}

#endif // TDMASOLVER_H
