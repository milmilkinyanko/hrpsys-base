// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  PreviewController.h
 * @brief
 *
 * Computes center of mass trajectory from reference ZMP:
 * Minimize
 * \f[
 * J = \Sum_i {Q(refZMP_i - ZMP_i}^2 + R(u_i)^2}
 * \f]
 *
 * Q, R : Output and Input weight
 * u : Input (COM jerk)
 *
 * Based on the preview control theory outlined in this paper:
 * "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point
 * by Shuuji Kajita et al.
 *
 * @date  $Date$
 */

#ifndef PREVIEWCONTROLLER_H
#define PREVIEWCONTROLLER_H

#include <iostream>
#include <vector>
#include <deque>
#include "DiscreteAlgebraicRiccatiEquation.h"

namespace hrp {

template<size_t dim_state>
class PreviewControllerBase
{
protected:
    static constexpr double DEFAULT_GRAVITATIONAL_ACCELERATION = 9.80665; // [m/s^2]
    static constexpr double DEFAULT_PREVIEW_TIME = 1.6; // [s]
    static constexpr double DEFAULT_OUTPUT_WEIGHT = 1.0;
    static constexpr double DEFAULT_INPUT_WEIGHT = 1.0e-6;

    Eigen::Matrix<double, 3, 3> tcA;
    Eigen::Matrix<double, 3, 1> tcB;
    Eigen::Matrix<double, 1, 3> tcC;
    Eigen::Matrix<double, 3, 2> x_k; // x, y
    Eigen::MatrixXd K; // TODO: not use MatrixXd
    Eigen::Matrix<double, dim_state, dim_state> P; // Solution of Riccati equation
    double Q; // Output weight
    double R; // Input weight
    Eigen::MatrixXd A_minus_bKt; // for buffer
    double R_btPb_inv; // for buffer
    Eigen::VectorXd f_vec;

    double cog_z;
    size_t preview_window;

    virtual void calc_f() = 0;
    virtual Eigen::Matrix<double, 1, 2> calc_u(const std::deque<Eigen::Vector3d>& ref_zmp) = 0; // jerk
    virtual void calc_x_k(const std::deque<Eigen::Vector3d>& ref_zmp) = 0;
    virtual void initMatrix() = 0;

    void initRiccati(const Eigen::Ref<const Eigen::Matrix<double, dim_state, dim_state>>& A,
                     const Eigen::Ref<const Eigen::Matrix<double, dim_state, 1>>& B,
                     const Eigen::Ref<const Eigen::Matrix<double, 1, dim_state>>& C)
    {
        const Eigen::Matrix<double, dim_state, dim_state> _Q(Q * C.transpose() * C);
        const Eigen::Matrix<double, 1, 1> _R(R);

        P = solveDiscreteAlgebraicRiccati<dim_state, 1>(A, B, _Q, _R);
        R_btPb_inv = (1.0 / (R + (B.transpose() * P * B)(0, 0)));
        K = R_btPb_inv * B.transpose() * P * A;
        A_minus_bKt = (A - B * K).transpose();
        calc_f();
    }

    /* inhibit copy constructor and copy insertion not by implementing */
    PreviewControllerBase(const PreviewControllerBase& _p);
    PreviewControllerBase &operator=(const PreviewControllerBase &_p);

public:
    /* dt = [s], zc = [m], d = [s] */
    PreviewControllerBase(const double dt, const double zc,
                          const Eigen::Vector3d& init_xk, const double _g_acc,
                          const double out_weight = DEFAULT_OUTPUT_WEIGHT,
                          const double in_weight = DEFAULT_INPUT_WEIGHT,
                          const double preview_time = DEFAULT_PREVIEW_TIME)
        : x_k(Eigen::Matrix<double, 3, 2>::Zero()),
          Q(out_weight), R(in_weight),
          cog_z(zc),
          preview_window(static_cast<size_t>(std::round(preview_time / dt)))
    {
        tcA << 1, dt, 0.5 * dt * dt,
            0, 1,  dt,
            0, 0,  1;
        tcB << 1 / 6.0 * dt * dt * dt,
            0.5 * dt * dt,
            dt;
        tcC << 1.0, 0.0, -zc / _g_acc;
        x_k(0, 0) = init_xk(0);
        x_k(0, 1) = init_xk(1);
    }

    void update_zc(const double zc,
                   const double _g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION)
    {
        cog_z = zc;
        tcC(0, 2) = -zc / _g_acc;
        initMatrix();
    }

    void set_x_k(const Eigen::Matrix<double, 3, 2>& _x_k) { x_k = _x_k; }

    size_t getPreviewWindow () { return preview_window; };
    double getPreview_f(const size_t idx) { return f_vec(idx); };
    Eigen::Vector3d getRefCog()    { return Eigen::Vector3d(x_k(0, 0), x_k(0, 1), cog_z); }
    Eigen::Vector3d getRefCogVel() { return Eigen::Vector3d(x_k(0, 0), x_k(0, 1), 0); }
    Eigen::Vector3d getRefCogAcc() { return Eigen::Vector3d(x_k(2, 0), x_k(2, 1), 0); }

    // TODO: move to COGTrajectoryGenerator
    // Eigen::Vector3d getCartZMP()
    // {
    //     const Eigen::Matrix<double, 1, 2> _p(tcC * x_k);
    //     return Eigen::Vector3d(_p(0, 0), _p(0, 1), ref_zmp_z.front()); // TODO
    // }

    const Eigen::Matrix<double, 3, 2>& getStateVector() const { return x_k; }
};

class PreviewController : public PreviewControllerBase<3>
{
private:
    void calc_f()
    {
        f_vec.resize(preview_window);
        Eigen::Matrix<double, 3, 3> gsi(Eigen::Matrix<double, 3, 3>::Identity());
        for (size_t i = 0; i < preview_window; ++i) {
            f_vec(i) = (R_btPb_inv * tcB.transpose() * (gsi * Q * tcC.transpose()))(0, 0);
            gsi = A_minus_bKt * gsi;
        }
    }

    Eigen::Matrix<double, 1, 2> calc_u(const std::deque<Eigen::Vector3d>& ref_zmp)
    {
        // assert ref_zmp.size() >= preview_window
        Eigen::Matrix<double, 1, 2> gfp(Eigen::Matrix<double, 1, 2>::Zero());\
        for (size_t i = 0; i < preview_window; ++i) {
            gfp += f_vec(i) * ref_zmp[i].head<2>(); // TODO: transposeいらない？ ref_zmpは0から？
        }
        return -K * x_k + gfp;
    }

    void calc_x_k(const std::deque<Eigen::Vector3d>& ref_zmp)
    {
        x_k = tcA * x_k + tcB * calc_u(ref_zmp);
    }

    void initMatrix()
    {
        initRiccati(tcA, tcB, tcC);
    }

public:
    PreviewController(const double dt, const double zc,
                      const Eigen::Vector3d& init_xk,
                      const double out_weight = DEFAULT_OUTPUT_WEIGHT,
                      const double in_weight = DEFAULT_INPUT_WEIGHT,
                      const double preview_time = DEFAULT_PREVIEW_TIME,
                      const double _g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION)
        : PreviewControllerBase(dt, zc, init_xk, _g_acc, out_weight, in_weight, preview_time)
        {
            initMatrix();
        }
};

class ExtendedPreviewController : public PreviewControllerBase<4>
{
private:
    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 1> B;
    Eigen::Matrix<double, 1, 4> C;
    Eigen::Matrix<double, 4, 2> x_k_e;

    void calc_f()
    {
        f_vec.resize(preview_window);
        Eigen::Matrix<double, 4, 4> gsi(Eigen::Matrix<double, 4, 4>::Identity());
        Eigen::Matrix<double, 4, 1> qt(Q * C.transpose());
        for (size_t i = 0; i < preview_window; ++i) {
            if (i == preview_window - 1) qt = P * C.transpose();
            f_vec(i) = (R_btPb_inv * B.transpose() * (gsi * qt))(0, 0);
            gsi = A_minus_bKt * gsi;
        }
    }

    Eigen::Matrix<double, 1, 2> calc_u(const std::deque<Eigen::Vector3d>& ref_zmp)
    {
        // TODO: assert ref_zmp.size() >= preview_window
        Eigen::Matrix<double, 1, 2> gfp(Eigen::Matrix<double, 1, 2>::Zero());
        for (size_t i = 0; i < preview_window; ++i) {
            gfp += f_vec(i) * ref_zmp[i].head<2>();
        }
        return -K * x_k_e + gfp;
    }

    void initMatrix()
    {
        const Eigen::Matrix<double, 1, 3> tmpca(tcC * tcA);
        const Eigen::Matrix<double, 1, 1> tmpcb(tcC * tcB);
        A << 1.0, tmpca(0,0), tmpca(0,1), tmpca(0,2),
             0.0, tcA(0,0), tcA(0,1), tcA(0,2),
             0.0, tcA(1,0), tcA(1,1), tcA(1,2),
             0.0, tcA(2,0), tcA(2,1), tcA(2,2);
        B << tmpcb(0, 0),
             tcB(0 ,0),
             tcB(1, 0),
             tcB(2, 0);
        C << 1, 0, 0, 0;
        initRiccati(A, B, C);
    }

public:
    ExtendedPreviewController(const double dt, const double zc,
                              const Eigen::Vector3d& init_xk,
                              const double out_weight = DEFAULT_OUTPUT_WEIGHT,
                              const double in_weight = DEFAULT_INPUT_WEIGHT,
                              const double preview_time = DEFAULT_PREVIEW_TIME,
                              const double _g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION)
        : PreviewControllerBase(dt, zc, init_xk, _g_acc, out_weight, in_weight, preview_time)
    {
        x_k_e.setZero();
        x_k_e(0, 0) = init_xk(0);
        x_k_e(0, 1) = init_xk(1);
        initMatrix();
    }

    void calc_x_k(const std::deque<Eigen::Vector3d>& ref_zmp)
    {
        x_k_e = A * x_k_e + B * calc_u(ref_zmp);
        x_k += x_k_e.block<3, 2>(1, 0);
    }

    const Eigen::Matrix<double, 4, 2>& getErrorState() const { return x_k_e; }
};

}

#endif // PREVIEWCONTROLLER_H
