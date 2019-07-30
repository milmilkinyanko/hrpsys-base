// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  FootStepType.h
 * @brief
 * @date  $Date$
 */

#ifndef FOOTSTEPTYPE_H
#define FOOTSTEPTYPE_H

namespace hrp
{

enum class LegType {RLEG, LLEG, RARM, LARM};

class contact_constraint
{
    // TODO: 拘束条件を持つ
    //       zmp offsetどうする？
  private:
    int link_id;
    std::vector<hrp::Vector3> link_contact_points; // TODO: local
    rats::coordinates link_representative_coord;
    rats::coordinates env_representative_coord;

    // TODO: 必要かもしれない変数たち
    hrp::Vector3 zmp_offset; // zmpに関するものはここじゃない？
    double weight;

  public:
    contact_constraint(const int _link_id) : link_id(_link_id) {}
    virtual ~contact_constraint() {}

    int getLinkId() const { return link_id; }
    const std::vector<hrp::Vector3>& getLinkContactPoints() const { return link_contact_points; }

    rats::coordinates& envRepresentativeCoord() { return env_representative_coord; }
    const rats::coordinates& envRepresentativeCoord() const { return env_representative_coord; }

    void addLinkContactPoint(const hrp::Vector3& pos)
    {
        link_contact_points.push_back(pos);
    }

    void clearLinkContactPoints()
    {
        link_contact_points.clear();
    }

    void calcEnvironmentRepresentativeCoordFromContacts(const std::vector<rats::coordinates>& coords);

    rats::coordinates calcRepresentativeCoord(const std::vector<hrp::Vector3>& points, const hrp::Matrix33& rot);

    void rotateLinkRepresentativeCoord(const hrp::Matrix33& rot);
};

struct step_node
{
    LegType l_r;
    coordinates worldcoords;
    double step_height;
    double step_time;

    step_node () : l_r(RLEG), worldcoords(coordinates()),
                   step_height(), step_time(),
                   toe_angle(), heel_angle(){};
    step_node (const leg_type _l_r, const coordinates& _worldcoords,
               const double _step_height, const double _step_time,
               const double _toe_angle, const double _heel_angle)
        : l_r(_l_r), worldcoords(_worldcoords),
          step_height(_step_height), step_time(_step_time),
          toe_angle(_toe_angle), heel_angle(_heel_angle) {};
    step_node (const std::string& _l_r, const coordinates& _worldcoords,
               const double _step_height, const double _step_time,
               const double _toe_angle, const double _heel_angle)
        : l_r((_l_r == "rleg") ? RLEG :
              (_l_r == "rarm") ? RARM :
              (_l_r == "larm") ? LARM :
              LLEG), worldcoords(_worldcoords),
          step_height(_step_height), step_time(_step_time),
          toe_angle(_toe_angle), heel_angle(_heel_angle) {};

    friend std::ostream &operator<<(std::ostream &os, const step_node &sn)
    {
        os << "footstep" << std::endl;
        os << "  name = [" << ((sn.l_r==LLEG)?std::string("lleg"):
                               (sn.l_r==RARM)?std::string("rarm"):
                               (sn.l_r==LARM)?std::string("larm"):
                               std::string("rleg")) << "]" << std::endl;
        os << "  pos =";
        os << (sn.worldcoords.pos).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
        os << "  rot =";
        os << (sn.worldcoords.rot).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
        os << "  step_height = " << sn.step_height << "[m], step_time = " << sn.step_time << "[s], "
           << "toe_angle = " << sn.toe_angle << "[deg], heel_angle = " << sn.heel_angle << "[deg]";
        return os;
    };
};

struct footstep_parameter
{
    /* translate position of a leg from default foot_midcoords
     *   vector -> (list rleg-pos[m] lleg-pos[m] )
     */
    std::vector<hrp::Vector3> leg_default_translate_pos;
    /* stride params indicate max stride */
    double stride_fwd_x; // [m]
    double stride_outside_y; // [m]
    double stride_outside_theta; // [deg]
    double stride_bwd_x; // [m]
    double stride_inside_y; // [m]
    double stride_inside_theta; // [deg]

    footstep_parameter (const std::vector<hrp::Vector3>& _leg_pos,
                        const double _stride_fwd_x, const double _stride_outside_y, const double _stride_outside_theta,
                        const double _stride_bwd_x, const double _stride_inside_y, const double _stride_inside_theta)
        : leg_default_translate_pos(_leg_pos),
          stride_fwd_x(_stride_fwd_x), stride_outside_y(_stride_outside_y), stride_outside_theta(_stride_outside_theta),
          stride_bwd_x(_stride_bwd_x), stride_inside_y(_stride_inside_y), stride_inside_theta(_stride_inside_theta) {};
};

}

#endif // FOOTSTEPTYPE_H
