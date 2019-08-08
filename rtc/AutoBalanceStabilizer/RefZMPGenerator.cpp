// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  RefZMPGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <iterator>
#include "REFZMPGenerator.h"

namespace hrp {


hrp::Vector3 refZMPGenerator::calcRefZMPUsingConstraint(const constraint_count_pairs& contact_pair)
{
    hrp::Vector3 zmp_pos = hrp::Vector3::Zero();
    double sum_weight = 0;
    for (const LinkConstraint& constraint : constraint_pair.first) {
        const double weight = constraint.getWeight();
        sum_pos += constraint.getLinkRepresentativePoint() * weight; // TODO: representative pointを使うかどうか
        sum_weight += weight;
    }
    zmp_pos /= sum_weight;

    return zmp_pos;
}

// Push or just calc
hrp::Vector3 refZMPGenerator::calcRefZMPUsingConstraintList(const std::vector<constraint_count_pairs>& constraint_list
                                                    const size_t count)
{
    size_t constraint_index = 0;
    for (size_t i = 0; i < constraint_list.size(); ++i) {
        if (count < constraint_list[i].second) break;
        constraint_index = i;
    }

    return calcRefZMPUsingConstraint(constraint_list[constraint_index]);
}

// No interpolation
void refZMPGenerator::setRefZMPListUsingConstraintList(const std::vector<constraint_count_pairs>& constraint_list,
                                                       const size_t list_size,
                                                       const size_t start_index)
{
    refzmp_list.resize(list_size);

    size_t count = constraint_list[0].second;
    size_t constraint_index = 0;
    size_t zmp_index = start_index;
    const size_t constraint_size = constraint_list.size();
    const size_t max_count = count + list_size;
    count += start_index;

    // TODO: zmp offset, flight phase
    for (size_t constraint_index = 0; count < max_count && constraint_index < constraint_size; ++constraint_index) {
        if (constraint_list[constraint_index].second <= count) continue;

        const hrp::Vector3 zmp_pos = calcRefZMPUsingConstraint(constraint_list[constraint_index]);
        // TODO: for消せそう
        for (; count < constraint_list[constraint_index].second && count < max_count; ++zmp_index, ++count) {
            refzmp_list[zmp_index] = zmp_pos;
        }
    }

    if (count < max_count && zmp_index > 0) {
        std::vector<hrp::Vector3>::const_iterator it = refzmp_list.begin();
        std::advance(it, zmp_index);
        std::fill(it, refzmp_list.end(), refzmp_list[zmp_index - 1]);
    }
}











void refzmp_generator::push_refzmp_from_footstep_nodes_for_dual(const std::vector<step_node>& fns,
                                                                const std::vector<step_node>& _support_leg_steps,
                                                                const std::vector<step_node>& _swing_leg_steps)
{
    std::vector<hrp::Vector3> dzl;
    const hrp::Vector3 tmp_zero = hrp::Vector3::Zero();
    std::vector<hrp::Vector3> foot_x_axises;
    double sum_of_weight = 0.0;

    for (const step_node& support_step : _support_leg_steps) {
        dzl.push_back((support_step.worldcoords.rot * default_zmp_offsets[support_step.l_r] + support_step.worldcoords.pos) * zmp_weight_map[support_step.l_r]);
        sum_of_weight += zmp_weight_map[support_step.l_r];
    }

    for (const step_node& swing_step : _swing_leg_steps) {
        dzl.push_back((swing_step.worldcoords.rot * default_zmp_offsets[swing_step.l_r] + swing_step.worldcoords.pos) * zmp_weight_mapn[swing_step.l_r]);
        sum_of_weight += zmp_weight_map[swing_step.l_r];
        foot_x_axises.push_back(swing_step.worldcoords.rot * hrp::Vector3::UnitX()); // TODO: use col(0)
    }

    foot_x_axises_list.push_back(foot_x_axises);
    const hrp::Vector3 ref_zmp = std::accumulate(dzl.begin(), dzl.end(), tmp_zero) / sum_of_weight;
    refzmp_cur_list.push_back(ref_zmp);
    std::vector<leg_type> swing_leg_types;
    for (size_t i = 0; i < fns.size(); i++) {
        swing_leg_types.push_back(fns.at(i).l_r);
    }
    swing_leg_types_list.push_back(swing_leg_types);
    step_count_list.push_back(static_cast<size_t>(fns.front().step_time / dt));
    toe_heel_types_list.emplace_back(SOLE, SOLE);
    //std::cerr << "double " << (fns[fs_index].l_r==RLEG?LLEG:RLEG) << " [" << refzmp_cur_list.back()(0) << " " << refzmp_cur_list.back()(1) << " " << refzmp_cur_list.back()(2) << "]" << std::endl;
}

void refzmp_generator::push_refzmp_from_footstep_nodes_for_single(const std::vector<step_node>& fns,
                                                                  const std::vector<step_node>& _support_leg_steps,
                                                                  const toe_heel_types& tht)
{
    // support leg = prev fns l_r
    // swing leg = fns l_r
    hrp::Vector3 ref_zmp;
    const hrp::Vector3 tmp_zero = hrp::Vector3::Zero(); // TODO: delete
    std::vector<hrp::Vector3> dzl;
    std::vector<hrp::Vector3> foot_x_axises;
    double sum_of_weight = 0.0;

    for (std::vector<step_node>::const_iterator it = _support_leg_steps.begin(); it != _support_leg_steps.end(); it++) {
        dzl.push_back((it->worldcoords.rot * default_zmp_offsets[it->l_r] + it->worldcoords.pos) * zmp_weight_map[it->l_r]);
        sum_of_weight += zmp_weight_map[it->l_r];
        foot_x_axises.push_back( hrp::Vector3(it->worldcoords.rot * hrp::Vector3::UnitX()) );
    }
    ref_zmp = std::accumulate(dzl.begin(), dzl.end(), tmp_zero) / sum_of_weight;
    refzmp_cur_list.push_back(ref_zmp);
    foot_x_axises_list.push_back(foot_x_axises);
    std::vector<leg_type> swing_leg_types;
    for (size_t i = 0; i< fns.size(); i++) {
        swing_leg_types.push_back(fns.at(i).l_r);
    }
    swing_leg_types_list.push_back( swing_leg_types );
    step_count_list.push_back(static_cast<size_t>(fns.front().step_time / dt));
    toe_heel_types_list.push_back(tht);
    //std::cerr << "single " << fns[fs_index-1].l_r << " [" << refzmp_cur_list.back()(0) << " " << refzmp_cur_list.back()(1) << " " << refzmp_cur_list.back()(2) << "]" << std::endl;
}

void refzmp_generator::calc_current_refzmp(std::vector<hrp::Vector3>& swing_foot_zmp_offsets,
                                           const double default_double_support_ratio_before,
                                           const double default_double_support_ratio_after,
                                           const double default_double_support_static_ratio_before,
                                           const double default_double_support_static_ratio_after)
{
    const size_t cnt = one_step_count - refzmp_count; // current counter (0 -> one_step_count)
    const size_t double_support_count_half_before = default_double_support_ratio_before * one_step_count;
    const size_t double_support_count_half_after = default_double_support_ratio_after * one_step_count;
    const size_t double_support_static_count_half_before = default_double_support_static_ratio_before * one_step_count;
    const size_t double_support_static_count_half_after = default_double_support_static_ratio_after * one_step_count;

    for (size_t i = 0; i < swing_leg_types_list[refzmp_index].size(); i++) {
        swing_foot_zmp_offsets.push_back(default_zmp_offsets[swing_leg_types_list[refzmp_index].at(i)]);
    }
    double zmp_diff = 0.0; // difference between total swing_foot_zmp_offset and default_zmp_offset

    // Calculate swing foot zmp offset for toe heel zmp transition
    if (use_toe_heel_transition &&
        !(is_start_double_support_phase() || is_end_double_support_phase())) { // Do not use toe heel zmp transition during start and end double support period because there is no swing foot
        double first_zmp_offset_x, second_zmp_offset_x;
        if (use_toe_heel_auto_set) {
            first_zmp_offset_x  = set_value_according_to_toe_heel_type(toe_heel_types_list[refzmp_index].src_type, toe_zmp_offset_x, heel_zmp_offset_x, swing_foot_zmp_offsets.front()(0));
            second_zmp_offset_x = set_value_according_to_toe_heel_type(toe_heel_types_list[refzmp_index].dst_type, toe_zmp_offset_x, heel_zmp_offset_x, swing_foot_zmp_offsets.front()(0));
        } else {
            first_zmp_offset_x = toe_zmp_offset_x;
            second_zmp_offset_x = heel_zmp_offset_x;
        }

        if (thp.is_between_phases(cnt, SOLE0)) {
            const double ratio = thp.calc_phase_ratio(cnt+1, SOLE0);
            swing_foot_zmp_offsets.front()(0) = (1-ratio)*swing_foot_zmp_offsets.front()(0) + ratio*first_zmp_offset_x;
        } else if (thp.is_between_phases(cnt, HEEL2SOLE, SOLE2)) {
            const double ratio = thp.calc_phase_ratio(cnt, HEEL2SOLE, SOLE2);
            swing_foot_zmp_offsets.front()(0) = ratio*swing_foot_zmp_offsets.front()(0) + (1-ratio)*second_zmp_offset_x;
        } else if (thp.is_between_phases(cnt, SOLE0, SOLE2TOE)) {
            swing_foot_zmp_offsets.front()(0) = first_zmp_offset_x;
        } else if (thp.is_between_phases(cnt, SOLE2HEEL, HEEL2SOLE)) {
            swing_foot_zmp_offsets.front()(0) = second_zmp_offset_x;
        } else if (thp.is_between_phases(cnt, SOLE2TOE, SOLE2HEEL)) {
            const double ratio = thp.calc_phase_ratio(cnt, SOLE2TOE, SOLE2HEEL);
            swing_foot_zmp_offsets.front()(0) = ratio * second_zmp_offset_x + (1-ratio) * first_zmp_offset_x;
        }

        zmp_diff = swing_foot_zmp_offsets.front()(0) - default_zmp_offsets[swing_leg_types_list[refzmp_index].front()](0);
        if ((is_second_phase() && ( cnt < double_support_count_half_before )) ||
            (is_second_last_phase() && ( cnt > one_step_count - double_support_count_half_after ))) {
            // "* 0.5" is for double supprot period
            zmp_diff *= 0.5;
        }
    }

    hrp::Vector3 ref_zmp;
    // Calculate total reference ZMP
    if (is_start_double_support_phase() || is_end_double_support_phase()) {
        ref_zmp = refzmp_cur_list[refzmp_index];
    } else if ( cnt < double_support_static_count_half_before ) { // Start double support static period
        const hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index];
        const hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index-1] + zmp_diff * foot_x_axises_list[refzmp_index-1].front();
        const double ratio = (is_second_phase() ? 1.0 : 0.5);
        ref_zmp = (1 - ratio) * current_support_zmp + ratio * prev_support_zmp;
    } else if ( cnt > one_step_count - double_support_static_count_half_after ) { // End double support static period
        const hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index+1] + zmp_diff * foot_x_axises_list[refzmp_index+1].front();
        const hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index];
        const double ratio = (is_second_last_phase() ? 1.0 : 0.5);
        ref_zmp = (1 - ratio) * prev_support_zmp + ratio * current_support_zmp;
    } else if ( cnt < double_support_count_half_before ) { // Start double support period
        const hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index];
        const hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index-1] + zmp_diff * foot_x_axises_list[refzmp_index-1].front();
        const double ratio = ((is_second_phase() ? 1.0 : 0.5) / (double_support_count_half_before-double_support_static_count_half_before)) * (double_support_count_half_before-cnt);
        ref_zmp = (1 - ratio) * current_support_zmp + ratio * prev_support_zmp;
    } else if ( cnt > one_step_count - double_support_count_half_after ) { // End double support period
        const hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index+1] + zmp_diff * foot_x_axises_list[refzmp_index+1].front();
        const hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index];
        const double ratio = ((is_second_last_phase() ? 1.0 : 0.5) / (double_support_count_half_after-double_support_static_count_half_after)) * (cnt - 1 - (one_step_count - double_support_count_half_after));
        ref_zmp = (1 - ratio) * prev_support_zmp + ratio * current_support_zmp;
    } else {
        ref_zmp = refzmp_cur_list[refzmp_index];
    }

    return ref_zmp;
}

void refzmp_generator::update_refzmp ()
{
    if ( 1 <= refzmp_count ) {
        refzmp_count--;
    } else {
        refzmp_index++;
        // Check length of step_count_list and refzmp_index
        //   The case if !(refzmp_index <= step_count_list.size()-1) is finalizing of gait_generator.
        //   If finalizing, this can be neglected.
        if (refzmp_index <= step_count_list.size() - 1) {
            refzmp_count = one_step_count = step_count_list[refzmp_index];
            thp.set_one_step_count(one_step_count);
        }
        //std::cerr << "fs " << fs_index << "/" << fnl.size() << " rf " << refzmp_index << "/" << refzmp_cur_list.size() << " flg " << std::endl;
    }
}

}
