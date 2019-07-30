// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  RefZMPGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef REFZMPGENERATOR_H
#define REFZMPGENERATOR_H

#ifdef FOR_TESTGAITGENERATOR
#warning "Compile for testGaitGenerator"
#endif // FOR_TESTGAITGENERATOR

#include <vector>
#include <memory>
#include <hrpUtil/EigenTypes.h>
#include "FootStepType.h"

namespace hrp
{

/* refzmp_generator to generate current refzmp from footstep_node_list */
class refZMPGenerator
{
  private:
    std::vector<hrp::Vector3> refzmp_list;
    std::vector<hrp::Vector3> default_zmp_offsets; /* list of RLEG and LLEG */
    std::unique_ptr<interpolator> zmp_weight_interpolator;

    void calcRefZMPUsingContactList(const contact_count_pairs& contact_points, const double dt, const size_t zmp_index);
    void calcRefZMPListUsingContactList(const contact_count_pairs& contact_points, const double dt, const size_t list_size);

  public:
    refZMPGenerator() {}
    virtual ~refZMPGenerator() {}

    const std::vector<hrp::Vector3>& getRefZMPList() const { return refzmp_list; }


















    void calc_current_refzmp(std::vector<hrp::Vector3>& swing_foot_zmp_offsets,
                             const double default_double_support_ratio_before,
                             const double default_double_support_ratio_after,
                             const double default_double_support_static_ratio_before,
                             const double default_double_support_static_ratio_after);

  public:
    refzmp_generator(const double _dt)
    {
        default_zmp_offsets.resize(4, hrp::Vector3::Zero());
        const double zmp_weight_initial_value[4] = {1.0, 1.0, 0.1, 0.1};
        zmp_weight_interpolator = std::make_unique<interpolator>(4, dt);
        zmp_weight_interpolator->set(zmp_weight_initial_value);
        zmp_weight_interpolator->setName("GaitGenerator zmp_weight_interpolator");
    }

    virtual ~refzmp_generator() {}

    void remove_refzmp_cur_list_over_length (const size_t len)
    {
        if (refzmp_cur_list.size() > len)      refzmp_cur_list.resize(len);
        if (foot_x_axises_list.size() > len)   foot_x_axises_list.resize(len);
        if (swing_leg_types_list.size() > len) swing_leg_types_list.resize(len);
        if (step_count_list.size() > len)      step_count_list.resize(len);
        if (toe_heel_types_list.size() > len)  toe_heel_types_list.resize(len);
    }

    void reset (const size_t _refzmp_count)
    {
        set_indices(0);
        one_step_count = _refzmp_count;
        set_refzmp_count(_refzmp_count);
        refzmp_cur_list.clear();
        foot_x_axises_list.clear();
        swing_leg_types_list.clear();
        step_count_list.clear();
        toe_heel_types_list.clear();
        thp.set_one_step_count(one_step_count);
    }

    void push_refzmp_from_footstep_nodes_for_dual(const std::vector<step_node>& fns,
                                                  const std::vector<step_node>& _support_leg_steps,
                                                  const std::vector<step_node>& _swing_leg_steps);
    void push_refzmp_from_footstep_nodes_for_single(const std::vector<step_node>& fns,
                                                    const std::vector<step_node>& _support_leg_steps,
                                                    const toe_heel_types tht);
    void update_refzmp();

    // setter
    void set_indices (const size_t idx) { refzmp_index = idx; };
    void set_refzmp_count(const size_t _refzmp_count) { refzmp_count = _refzmp_count; };
    void set_default_zmp_offsets(const std::vector<hrp::Vector3>& tmp) { default_zmp_offsets = tmp; };
    void set_toe_zmp_offset_x (const double _off) { toe_zmp_offset_x = _off; };
    void set_heel_zmp_offset_x (const double _off) { heel_zmp_offset_x = _off; };
    void set_use_toe_heel_transition (const bool _u) { use_toe_heel_transition = _u; };
    void set_use_toe_heel_auto_set (const bool _u) { use_toe_heel_auto_set = _u; };
    void set_zmp_weight_map (const std::map<leg_type, double> _map) {
        double zmp_weight_array[4] = {_map.find(RLEG)->second, _map.find(LLEG)->second, _map.find(RARM)->second, _map.find(LARM)->second};
        if (zmp_weight_interpolator->isEmpty()) {
            zmp_weight_interpolator->clear();
            double zmp_weight_initial_value[4] = {zmp_weight_map[RLEG], zmp_weight_map[LLEG], zmp_weight_map[RARM], zmp_weight_map[LARM]};
            zmp_weight_interpolator->set(zmp_weight_initial_value);
            zmp_weight_interpolator->setGoal(zmp_weight_array, 2.0, true);
        } else {
            std::cerr << "zmp_weight_map cannot be set because interpolating." << std::endl;
        }
    }
    bool set_toe_heel_phase_ratio (const std::vector<double>& ratio) { return thp.set_toe_heel_phase_ratio(ratio); }

    // getter
    bool get_current_refzmp(std::vector<hrp::Vector3>& swing_foot_zmp_offsets,
                            const double default_double_support_ratio_before,
                            const double default_double_support_ratio_after,
                            const double default_double_support_static_ratio_before,
                            const double default_double_support_static_ratio_after)
    {
        if (refzmp_cur_list.size() > refzmp_index ) calc_current_refzmp(swing_foot_zmp_offsets, default_double_support_ratio_before, default_double_support_ratio_after, default_double_support_static_ratio_before, default_double_support_static_ratio_after);
        return refzmp_cur_list.size() > refzmp_index;
    }
    hrp::Vector3 get_refzmp_cur () const { return refzmp_cur_list.front(); }
    hrp::Vector3 get_default_zmp_offset (const leg_type lt) const { return default_zmp_offsets[lt]; }
    double get_toe_zmp_offset_x () const { return toe_zmp_offset_x; }
    double get_heel_zmp_offset_x () const { return heel_zmp_offset_x; }
    bool get_use_toe_heel_transition () const { return use_toe_heel_transition; }
    bool get_use_toe_heel_auto_set () const { return use_toe_heel_auto_set; }
    const std::map<leg_type, double> get_zmp_weight_map () const { return zmp_weight_map; }
    void proc_zmp_weight_map_interpolation ()
    {
        if (zmp_weight_interpolator->isEmpty()) return;
        double zmp_weight_output[4];
        zmp_weight_interpolator->get(zmp_weight_output, true);
        zmp_weight_map = boost::assign::map_list_of<leg_type, double>
            (RLEG, zmp_weight_output[0])(LLEG, zmp_weight_output[1])
            (RARM, zmp_weight_output[2])(LARM, zmp_weight_output[3])
            .convert_to_container<std::map<leg_type, double> > ();
    }
#ifdef FOR_TESTGAITGENERATOR
    std::vector<hrp::Vector3> get_default_zmp_offsets() const { return default_zmp_offsets; };
#endif // FOR_TESTGAITGENERATOR
};

}

#endif // REFZMPGENERATOR_H
