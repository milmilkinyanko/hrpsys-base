#ifndef FULLBODYIK_H
#define FULLBODYIK_H

#include <iomanip>
#include <cfloat>
#include <array>
#include <hrpModel/JointPath.h>
#include "../ImpedanceController/RatsMatrix.h"
#include "../TorqueFilter/IIRFilter.h"
#include "Utility.h"

#define OPENHRP_PACKAGE_VERSION_320

namespace hrp {

struct IKConstraint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string target_link_name; // TODO: enumでCOMを指定して，target_nameはidにする
    hrp::Vector3 targetPos = hrp::Vector3::Zero();
    hrp::Vector3 targetOmega = hrp::Vector3::Zero(); // TODO
    hrp::Vector3 localPos = hrp::Vector3::Zero();
    hrp::Matrix33 localR = hrp::Matrix33::Identity();
    hrp::dvector6 constraint_weight = hrp::dvector6::Ones();
    double pos_precision = 1e-4;
    double rot_precision = deg2rad(0.1);

    hrp::Vector3 calcWorldPos(const hrp::BodyPtr& _robot)
    {
        const hrp::Link* const target_link = _robot->link(target_link_name);
        return target_link->p + target_link->R * localPos;
    }

    hrp::Matrix33 calcWorldRot(const hrp::BodyPtr& _robot)
    {
        return _robot->link(target_link_name)->R * localR;
    }
};


class FullbodyInverseKinematicsSolver
{
  private:
    constexpr static size_t WS_DOF = 6;
    constexpr static size_t BASE_DOF = 6;
    const size_t J_DOF;
    const size_t ALL_DOF;
    const double m_dt;
    std::array<IIRFilter, 3> am_filters;
    hrp::dvector jlim_avoid_weight_old;
    hrp::dvector q_ref;

    inline hrp::dmatrix toSelectionMatrix(const hrp::dvector& in)
    {
        hrp::dmatrix ret = hrp::dmatrix::Zero((in.array() > 0.0).count(), in.size());
        if (ret.rows() != 0 && ret.cols() != 0) {
            for (int row=0, col=0; col< in.size(); ++col) {
                if(in(col) > 0.0){
                    ret(row, col) = 1;
                    row++;
                }
            }
        }
        return ret;
    }

    inline hrp::dvector getRobotStates(const hrp::BodyPtr& _robot)
    {
        hrp::dvector joint_angles;
        hrp::copyJointAnglesFromRobotModel(joint_angles, _robot);
        hrp::dvector robot_states(_robot->numJoints() + 6);
        robot_states << joint_angles, _robot->rootLink()->p, hrp::omegaFromRot(_robot->rootLink()->R);
        return robot_states;
    }

    bool checkIKConvergence(const std::vector<IKConstraint>& _ikc_list)
    {
        for (size_t i = 0, ikc_size = _ikc_list.size(); i < ikc_size; ++i) {
            const hrp::Link* const link_tgt_ptr = m_robot->link(_ikc_list[i].target_link_name);
            hrp::Vector3 pos_err, rot_err;
            if (link_tgt_ptr) {
                pos_err = _ikc_list[i].targetPos - (link_tgt_ptr->p + link_tgt_ptr->R * _ikc_list[i].localPos);
                rats::difference_rotation(rot_err, (link_tgt_ptr->R * _ikc_list[i].localR), calcRodriguesFromOmega(_ikc_list[i].targetOmega));
            } else if (!link_tgt_ptr && _ikc_list[i].target_link_name == "COM") {
                pos_err = _ikc_list[i].targetPos - (m_robot->calcCM() + _ikc_list[i].localR * _ikc_list[i].localPos);
                rot_err = _ikc_list[i].targetOmega - cur_momentum_around_COM;
            } else {
                std::cerr << "Unknown Link Target !!" << std::endl;
                continue;
            }

            for (size_t j = 0; j < 3; ++j) {
                if (_ikc_list[i].constraint_weight[j]     > 0 && fabs(pos_err(j)) > _ikc_list[i].pos_precision) return false;
                if (_ikc_list[i].constraint_weight[j + 3] > 0 && fabs(rot_err(j)) > _ikc_list[i].rot_precision) return false;
            }
        }
        return true;
    }

    inline hrp::Matrix33 calcRodriguesFromOmega(const hrp::Vector3& omega) {
        // To avoid zero division when normalizing. This problem has been resolved in the latest version of Eigen.
        const double omega_norm = omega.norm();
        const hrp::Vector3 omega_normalized = omega_norm > 1e-4 ? omega.normalized() : omega;
        return hrp::rodrigues(omega_normalized, omega_norm);
    }

  public:
    hrp::BodyPtr m_robot;
    hrp::dvector dq_weight_all, q_ref_max_dq, q_ref_constraint_weight;
    hrp::Vector3 cur_momentum_around_COM = hrp::Vector3::Zero();
    hrp::Vector3 cur_momentum_around_COM_filtered = hrp::Vector3::Zero();
    hrp::Vector3 rootlink_rpy_llimit = hrp::Vector3::Constant(-DBL_MAX);
    hrp::Vector3 rootlink_rpy_ulimit = hrp::Vector3::Constant(DBL_MAX);
    FullbodyInverseKinematicsSolver(const hrp::BodyPtr& _robot, const std::string& _print_str, const double _dt)
        : m_robot(_robot),
          m_dt(_dt),
          J_DOF(_robot->numJoints()),
          ALL_DOF(J_DOF + BASE_DOF)
    {
        dq_weight_all = hrp::dvector::Ones(ALL_DOF);
        q_ref = q_ref_constraint_weight = hrp::dvector::Zero(ALL_DOF);
        q_ref_max_dq = hrp::dvector::Constant(ALL_DOF, 1e-2);
        jlim_avoid_weight_old = hrp::dvector::Constant(ALL_DOF, 0);

        for(size_t i = 0; i < 3; ++i) {
            am_filters[i].setParameterAsBiquad(10, 1/std::sqrt(2), 1.0/m_dt);
            am_filters[i].reset();
        }
    }

    size_t numStates() const { return ALL_DOF; };

    void setReferenceRobotStatesFromBody(const hrp::BodyPtr& _robot) { q_ref = getRobotStates(_robot); }

    int solveFullbodyIKLoop (const std::vector<IKConstraint>& _ikc_list, const int _max_iteration)
    {
#ifdef OPENHRP_PACKAGE_VERSION_320
        const hrp::Vector3 base_p_old = m_robot->rootLink()->p;
        const hrp::Matrix33 base_R_old = m_robot->rootLink()->R;
        hrp::dvector q_old(J_DOF);
        copyJointAnglesFromRobotModel(q_old, m_robot);

        int loop = 0;
        while (loop < _max_iteration) {
            solveFullbodyIKOnce(_ikc_list);

            //check ang moment
            m_robot->rootLink()->v = (m_robot->rootLink()->p - base_p_old) / m_dt;
            m_robot->rootLink()->w = base_R_old * hrp::omegaFromRot(base_R_old.transpose() * m_robot->rootLink()->R) / m_dt;
            for (size_t i = 0; i < J_DOF; ++i) {
                m_robot->joint(i)->dq = (m_robot->joint(i)->q - q_old(i)) / m_dt;
            }
            m_robot->calcForwardKinematics(true, false);

            hrp::Vector3 tmp_P, tmp_L;
            m_robot->calcTotalMomentum(tmp_P, tmp_L); // calcTotalMomentumは漸化的にWorld周りの並進＋回転運動量を出す
            cur_momentum_around_COM = tmp_L - m_robot->calcCM().cross(tmp_P);
            // m_robot->calcTotalMomentumFromJacobian(tmp_P, cur_momentum_around_COM); //calcTotalMomentumFromJacobianは重心ヤコビアンと重心周り角運動量ヤコビアンを用いて重心周りの並進＋回転運動量を出す

            // TODO: Cause problems when max_iteration > 1
            // for (size_t i = 0; i < 3; ++i) {
            //     cur_momentum_around_COM_filtered(i) = am_filters[i].passFilter(cur_momentum_around_COM(i));
            // }

            ++loop;
            if (checkIKConvergence(_ikc_list)) break;
        }
        return loop;
#else
        std::cerr << "solveFullbodyIKLoop() needs OPENHRP_PACKAGE_VERSION_320 !!!" << std::endl;
        return -1;
#endif
    }

    void solveFullbodyIKOnce(const std::vector<IKConstraint>& _ikc_list)
    {
#ifdef OPENHRP_PACKAGE_VERSION_320

        // count up all valid constraint DOF and allocate Jacobian and vectors
        hrp::dmatrix q_select_mat = toSelectionMatrix(dq_weight_all);
        const size_t VALID_Q_NUM = q_select_mat.rows();
        if (VALID_Q_NUM == 0) return;

        size_t VALID_C_NUM = 0; // count up valid constraint num
         // count up valid end effector constraint num
        for (size_t i = 0; i < _ikc_list.size(); ++i) {
            VALID_C_NUM += (_ikc_list[i].constraint_weight.array() > 0.0).count();
        }
        VALID_C_NUM += (q_ref_constraint_weight.array() > 0.0).count(); // count up valid reference joint angle constraint num

        hrp::dmatrix J_all = hrp::dmatrix::Zero(VALID_C_NUM, VALID_Q_NUM);
        hrp::dvector err_all = hrp::dvector::Zero(VALID_C_NUM);
        hrp::dvector constraint_weight_all = hrp::dvector::Ones(VALID_C_NUM);

        // set end effector constraints into Jacobian
        size_t CURRENT_C_COUNT = 0;
        for (size_t i = 0; i < _ikc_list.size(); ++i) {
            hrp::Link* link_tgt_ptr = m_robot->link(_ikc_list[i].target_link_name);
            hrp::dmatrix J_part = hrp::dmatrix::Zero(WS_DOF, ALL_DOF); //全身to拘束点へのヤコビアン
            hrp::dvector6 dp_part; //pos + rot 速度ではなく差分

            //ベースリンク，通常リンク共通
            if (link_tgt_ptr) {
                const hrp::Vector3 tgt_cur_pos = link_tgt_ptr->p + link_tgt_ptr->R * _ikc_list[i].localPos;
                const hrp::Matrix33 tgt_cur_rot = link_tgt_ptr->R * _ikc_list[i].localR;
                dp_part.head<3>() =  _ikc_list[i].targetPos - tgt_cur_pos;
                dp_part.tail<3>() = tgt_cur_rot * hrp::omegaFromRot(tgt_cur_rot.transpose() * calcRodriguesFromOmega(_ikc_list[i].targetOmega));
                const hrp::JointPath tgt_jpath(m_robot->rootLink(), link_tgt_ptr);
                hrp::dmatrix J_jpath;
                tgt_jpath.calcJacobian(J_jpath, _ikc_list[i].localPos);

                //ジョイントパスのJacobianを全身用に並び替え
                for (size_t id_in_jpath = 0; id_in_jpath < tgt_jpath.numJoints(); ++id_in_jpath) {
                    J_part.col(tgt_jpath.joint(id_in_jpath)->jointId) = J_jpath.col(id_in_jpath);
                }

                J_part.rightCols(BASE_DOF) = hrp::dmatrix::Identity(WS_DOF,  BASE_DOF);
                J_part.rightCols(BASE_DOF).topRightCorner(3,3) = -hrp::hat(tgt_cur_pos - m_robot->rootLink()->p);
            } else if (!link_tgt_ptr && _ikc_list[i].target_link_name == "COM") {
                dp_part.head<3>() = _ikc_list[i].targetPos - m_robot->calcCM();
                // dp_part.tail<3>() = (_ikc_list[i].targetOmega - cur_momentum_around_COM_filtered) * m_dt;// COMのrotはAngulerMomentumとして扱う&差分なのでdtかける(+フィルタ)
                dp_part.tail<3>() = (_ikc_list[i].targetOmega - cur_momentum_around_COM) * m_dt;

                hrp::dmatrix J_com, J_am;
                m_robot->calcCMJacobian(nullptr, J_com); //デフォで右端に3x6のbase->COMのヤコビアンが付いてくる
                m_robot->calcAngularMomentumJacobian(nullptr, J_am); //base=NULLの時は重心周りの角運動量っぽい？
                J_part << J_com, J_am;
            } else {
                std::cerr << "Unknown Link Target !!" << std::endl;
                continue;
            }

            // set one of the end effector constraints into Jacobian
            const hrp::dmatrix c_part_selection_mat = toSelectionMatrix(_ikc_list[i].constraint_weight);   // select only valid end effector constraint
            if (c_part_selection_mat.rows() != 0 && c_part_selection_mat.cols() != 0) {
                const double dp_max = 0.1; // ???????

                J_all.middleRows             (CURRENT_C_COUNT, c_part_selection_mat.rows()) = c_part_selection_mat * J_part * q_select_mat.transpose();
                err_all.segment              (CURRENT_C_COUNT, c_part_selection_mat.rows()) = c_part_selection_mat * dp_part.cwiseMin(hrp::dvector6::Constant(dp_max)).cwiseMax(hrp::dvector6::Constant(-dp_max));
                constraint_weight_all.segment(CURRENT_C_COUNT, c_part_selection_mat.rows()) = c_part_selection_mat * _ikc_list[i].constraint_weight;
                CURRENT_C_COUNT += c_part_selection_mat.rows();
            }
        }

        // set reference joint angle constraints into Jacobian
        const hrp::dmatrix q_ref_selection_mat = toSelectionMatrix(q_ref_constraint_weight);
        if (q_ref_selection_mat.rows() != 0 && q_ref_selection_mat.cols() != 0) {
            J_all.middleRows             (CURRENT_C_COUNT, q_ref_selection_mat.rows()) = q_ref_selection_mat * hrp::dmatrix::Identity(ALL_DOF,ALL_DOF) * q_select_mat.transpose();
            err_all.segment              (CURRENT_C_COUNT, q_ref_selection_mat.rows()) = q_ref_selection_mat * ((q_ref - getRobotStates(m_robot)).cwiseMin(q_ref_max_dq).cwiseMax(-q_ref_max_dq));
            constraint_weight_all.segment(CURRENT_C_COUNT, q_ref_selection_mat.rows()) = q_ref_selection_mat * q_ref_constraint_weight;
            CURRENT_C_COUNT += q_ref_selection_mat.rows();
        }
        assert(CURRENT_C_COUNT == VALID_C_NUM);

        // joint limit avoidance (copy from JointPathEx)
        // TODO: このへんomegaだと無理かも
        hrp::dvector dq_weight_all_jlim = hrp::dvector::Ones(ALL_DOF);
        for (size_t j = 0; j < ALL_DOF; ++j) {
            double jang       = (j < J_DOF) ? m_robot->joint(j)->q      : hrp::omegaFromRot(m_robot->rootLink()->R)(j - J_DOF);
            const double jmax = (j < J_DOF) ? m_robot->joint(j)->ulimit : rootlink_rpy_ulimit(j - J_DOF);
            const double jmin = (j < J_DOF) ? m_robot->joint(j)->llimit : rootlink_rpy_llimit(j - J_DOF);

            constexpr double eps = deg2rad(1);
            if (eps_eq(jang, jmax, eps) && !eps_eq(jang, jmin, eps)) {
                jang = jmax - eps;
            } else if (!eps_eq(jang, jmax, eps) && eps_eq(jang, jmin, eps)) {
                jang = jmin + eps;
            }

            double jlim_avoid_weight;
            if (eps_eq(jang, jmax, eps) && eps_eq(jang, jmin, eps)) {
                jlim_avoid_weight = DBL_MAX;
            } else {
                jlim_avoid_weight = fabs( (pow((jmax - jmin),2) * (( 2 * jang) - jmax - jmin)) / (4 * pow((jmax - jang),2) * pow((jang - jmin),2)) );
                if (isnan(jlim_avoid_weight)) jlim_avoid_weight = 0;
            }

            if ((jlim_avoid_weight - jlim_avoid_weight_old(j)) >= 0) { // add weight only if q approaching to the limit
                dq_weight_all_jlim(j) += jlim_avoid_weight;
            }
            jlim_avoid_weight_old(j) = jlim_avoid_weight;
        }

        // const hrp::dvector dq_weight_all_final = q_select_mat * static_cast<hrp::dvector>(dq_weight_all.array() * dq_weight_all_jlim.array());
        hrp::dvector dq_weight_all_final = dq_weight_all.array() * dq_weight_all_jlim.array();
        dq_weight_all_final = q_select_mat * dq_weight_all_final;

        // Solvability-unconcerned Inverse Kinematics by Levenberg-Marquardt Method [Sugihara:JRSJ2011]
        constexpr double wn_const = 1e-3;
        const hrp::dmatrix Wn = dq_weight_all_final.asDiagonal() *
            (static_cast<double>(err_all.transpose() * constraint_weight_all.asDiagonal() * err_all) + wn_const) *
            hrp::dmatrix::Identity(dq_weight_all_final.size(), dq_weight_all_final.size());
        // Wn = dq_weight_all_final.asDiagonal() * Wn;
        const hrp::dmatrix H = J_all.transpose() * constraint_weight_all.asDiagonal() * J_all + Wn;
        const hrp::dvector g = J_all.transpose() * constraint_weight_all.asDiagonal() * err_all;
        hrp::dvector dq_all = H.ldlt().solve(g); // dq_all = H.inverse() * g; is slow

        // rtconf localhost:15005/wbms.rtc set debugLevel 1 とかにしたい
#if 0
#define dbg(var)  std::cout << #var"= " << (var) << std::endl
#define dbgn(var) std::cout << #var"= " << std::endl <<(var) << std::endl
#define dbgv(var) std::cout << #var"= " << (var.transpose()) << std::endl
        static size_t count = 0;
        if (count++ % 10000 == 0) {
            std::cout << std::setprecision(2) << "J=\n" << J_all << std::setprecision(6) << std::endl;
            dbg(J_all.rows());
            dbg(J_all.cols());
            dbgn(H);
            dbg(H.rows());
            dbg(H.cols());
            dbgn(Wn);
            dbg(Wn.rows());
            dbg(Wn.cols());
            dbgv(g);
            dbg(g.rows());
            dbg(g.cols());
            dbgv(err_all);
            dbg(err_all.rows());
            dbg(err_all.cols());
            dbgv(dq_all);
            dbg(dq_all.rows());
            dbg(dq_all.cols());
            dbgv(constraint_weight_all);
            dbg(constraint_weight_all.rows());
            dbg(constraint_weight_all.cols());
            dbgn(q_ref_selection_mat);
            dbgv(q_ref);
            dbgv(getRobotStates(m_robot));
            std::cout<<std::endl;
        }
#endif
        // update joint angles
        for (size_t i = 0; i < dq_all.rows(); ++i) {
            if (isnan(dq_all(i)) || isinf(dq_all(i))) {
                std::cerr <<"[FullbodyIK] ERROR nan/inf is found" << std::endl;
                return;
            }
        }

        dq_all = q_select_mat.transpose() * dq_all;
        for (size_t i = 0; i < J_DOF; ++i) {
            m_robot->joint(i)->q += dq_all(i);
            m_robot->joint(i)->q = hrp::clamp(m_robot->joint(i)->q, m_robot->joint(i)->llimit, m_robot->joint(i)->ulimit);
        }

        {
            // rootlink rpy limit ???
            // TODO: rpyじゃなくてomegaなのでは
            const hrp::Vector3 root_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
            for(size_t i = 0; i < 3; ++i) {
                // TODO: 0ではなくて上限まで動かすとかあるのでは,というのと，打ち切るのは最後の最後で良いのか
                if      (root_rpy[i] < rootlink_rpy_llimit[i] && dq_all.tail<3>()[i] < 0) dq_all.tail<3>()[i] = 0;
                else if (root_rpy[i] > rootlink_rpy_ulimit[i] && dq_all.tail<3>()[i] > 0) dq_all.tail<3>()[i] = 0;
            }
        }

        // update rootlink pos rot
        m_robot->rootLink()->p += dq_all.tail<6>().head<3>();
        const hrp::Vector3 omega = dq_all.tail<6>().tail<3>();
        const hrp::Matrix33 dR = calcRodriguesFromOmega(omega);
        rats::rotm3times(m_robot->rootLink()->R, dR, m_robot->rootLink()->R); // safe rot operation with quartanion normalization
        if (!m_robot->rootLink()->R.isUnitary()) {
            std::cerr <<"[FullbodyIK] WARN m_robot->rootLink()->R is not Unitary, something wrong !" << std::endl;
        }
        m_robot->calcForwardKinematics();
#else
        std::cerr<<"solveFullbodyIKOnce() needs OPENHRP_PACKAGE_VERSION_320 !!!"<<std::endl;
#endif
    }
};

}
#endif //  FULLBODYIK_H
