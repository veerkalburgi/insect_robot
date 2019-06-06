#ifndef LEG_IK_H
#define LEG_IK_H

#include <iostream>
#include <string>
#include <vector> 

#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "urdf/model.h"

namespace insect{

const double PI = 3.14159;
const double HIP_L = 0.148;
const double LIMB_L = 0.219;
const double HBDY_L = 0.3;
const int JNTS_NUM = 10;
enum leg_type {L_F, R_F, L_B, R_B};

class LegIK{
    public:
        LegIK(const std::string _name);
        LegIK(const std::string _name, bool _up2down, std::string _urdf_param, double _timeout, double _eps);
        void init();
        void setUp2Down(const bool _up2down); // set leg chain direction
        void setEndPose(const std::vector<double>& _pose); // x, y, z, R, P, Y
        bool getJntArray(std::vector<double>& _jnts);
        bool Up2Down(){return up2down_;}
    private:
        std::string name_;
        leg_type legt_;
        bool up2down_; // the chain direction: from base to shoe, or inverse
        std::string urdf_param_;
        double timeout_;
        double eps_;

        // base to shoe
        TRAC_IK::TRAC_IK* tracik_solver_ptr_;
        KDL::ChainFkSolverPos_recursive* fk_solver_ptr_;
        KDL::Chain chain_;
        KDL::JntArray jnt_array_; // current joint angle array
        KDL::JntArray ll_, ul_; // joint angle lower bound and upper bound
        std::string chain_start_;
        std::string chain_end_;
        std::vector<double> start2end_; // start to end transformation x y z R P Y, for setEndPose preprocess

        // shoe to base
        TRAC_IK::TRAC_IK* inv_tracik_solver_ptr_;
        KDL::ChainFkSolverPos_recursive* inv_fk_solver_ptr_;
        KDL::Chain inv_chain_;
        KDL::JntArray inv_jnt_array_; // current joint angle array
        KDL::JntArray inv_ll_, inv_ul_; // joint angle lower bound and upper bound
        std::string inv_chain_start_;
        std::string inv_chain_end_;
        std::vector<double> inv_start2end_; // start to end transformation x y z R P Y, for setEndPose preprocess

        KDL::Frame target_frame_;
};



} //namespace quadruped_9g

#endif
