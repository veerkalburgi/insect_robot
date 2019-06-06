#include "insect_robot_3/leg_ik.hpp"

namespace insect{

void LegIK::init()
{
    if(name_ == "l_f")
    {
        legt_ = L_F;
    }
    else if(name_ == "r_f")
    {
        legt_ = R_F;
    }
    else if(name_ == "l_b")
    {
        legt_ = L_B;
    }
    else if(name_ == "r_b")
    {
        legt_ = R_B;
    }

    // base to shoe
    chain_start_ = "torso_base";
    chain_end_ = name_ +"_limb";
    tracik_solver_ptr_ = new TRAC_IK::TRAC_IK(chain_start_, chain_end_, urdf_param_, timeout_, eps_);
    bool valid = tracik_solver_ptr_->getKDLChain(chain_);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
    }
    valid = tracik_solver_ptr_->getKDLLimits(ll_, ul_);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found");
    }
    fk_solver_ptr_ = new KDL::ChainFkSolverPos_recursive(chain_);
    jnt_array_ = KDL::JntArray(chain_.getNrOfJoints());
    std::vector<std::vector<double>> base2shoe {{HBDY_L, HIP_L, -LIMB_L, 0, 0, 0},
                                                {HBDY_L, -HIP_L, -LIMB_L, 0, 0, 0},
                                                {-HBDY_L, HIP_L, -LIMB_L, 0, 0, 0},
                                                {-HBDY_L, -HIP_L, -LIMB_L, 0, 0, 0}};
    jnt_array_(0) = 0; // shoulder joint
    jnt_array_(1) = -PI/3.0; // consider as limb joint
    jnt_array_(2) = -PI/3.0;
    jnt_array_(3) = -PI/3.0;
    jnt_array_(4) = 0; // shoe joint
    switch(legt_)
    {
        case L_F: start2end_ = base2shoe[0]; break;
        case R_F: start2end_ = base2shoe[1]; break;
        case L_B: start2end_ = base2shoe[2]; break;
        case R_B: start2end_ = base2shoe[3]; break;
    }


    // shoe to base
    inv_chain_start_ = name_ + "_limb";
    inv_chain_end_ = "torso_base";
    inv_tracik_solver_ptr_ = new TRAC_IK::TRAC_IK(inv_chain_start_, inv_chain_end_, urdf_param_, timeout_, eps_);
    valid = inv_tracik_solver_ptr_->getKDLChain(inv_chain_);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
    }
    valid = inv_tracik_solver_ptr_->getKDLLimits(inv_ll_, inv_ul_);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found");
    }
    inv_fk_solver_ptr_ = new KDL::ChainFkSolverPos_recursive(chain_);
    inv_jnt_array_ = KDL::JntArray(chain_.getNrOfJoints());
    std::vector<std::vector<double>> shoe2base {{-HBDY_L, -HIP_L, LIMB_L, 0, 0, 0},
                                                {-HBDY_L, HIP_L, LIMB_L, 0, 0, 0},
                                                {HBDY_L, -HIP_L, LIMB_L, 0, 0, 0},
                                                {HBDY_L, HIP_L, LIMB_L, 0, 0, 0}};
    inv_jnt_array_(0) = 0;
    inv_jnt_array_(1) = -PI/3.0;
    inv_jnt_array_(2) = -PI/3.0;
    inv_jnt_array_(3) = PI/6.0;
    inv_jnt_array_(4) = 0;
    switch(legt_)
    {
        case L_F: inv_start2end_ = shoe2base[0]; break;
        case R_F: inv_start2end_ = shoe2base[1]; break;
        case L_B: inv_start2end_ = shoe2base[2]; break;
        case R_B: inv_start2end_ = shoe2base[3]; break;
    }

}

LegIK::LegIK(const std::string _name):
    name_(_name),
    up2down_(true),
    urdf_param_("/robot_description"),
    timeout_(0.005),
    eps_(2e-5)
{
    init();
}

LegIK::LegIK(const std::string _name, bool _up2down, std::string _urdf_param, double _timeout, double _eps):
    name_(_name),
    up2down_(_up2down),
    urdf_param_(_urdf_param),
    timeout_(_timeout),
    eps_(_eps)
{
    init();
}

void LegIK::setUp2Down(const bool _up2down)
{
    up2down_ = _up2down;
}

void LegIK::setEndPose(const std::vector<double>& _pose) // x, y, z, R, P, Y
{
    std::vector<double> bias;
    if(up2down_)
    {
        bias = start2end_;
    }
    else
    {
        bias = inv_start2end_;
    }
    double x = _pose[0] + bias[0];
    double y = _pose[1] + bias[1];
    double z = _pose[2] + bias[2];
    double R = _pose[3] + bias[3];
    double P = _pose[4] + bias[4];
    double Y = _pose[5] + bias[5];
    KDL::Vector v(x, y, z);
    target_frame_ = KDL::Frame(KDL::Rotation::RPY(R, P, Y), v);
}

bool LegIK::getJntArray(std::vector<double>& _jnts)
{
    KDL::JntArray result;
    int rc = 0;
    if(up2down_)
    {
        rc = tracik_solver_ptr_->CartToJnt(jnt_array_, target_frame_, result);
        for(size_t i = 0, j = chain_.getNrOfJoints()-1; i < chain_.getNrOfJoints(); i ++, j--)
        {
            inv_jnt_array_(j) = jnt_array_(i);
        }
    }
    else
    {
        rc = inv_tracik_solver_ptr_->CartToJnt(inv_jnt_array_, target_frame_, result);
        for(size_t i = 0, j = chain_.getNrOfJoints()-1; i < chain_.getNrOfJoints(); i ++, j--)
        {
            jnt_array_(j) = inv_jnt_array_(i);
        }
    }
    if(rc < 0)
    {
        return false;
    }
    else
    {
        _jnts.clear();
        if(up2down_)
        {
            for(size_t i = 0; i < chain_.getNrOfJoints(); i ++)
            {
                _jnts.push_back(result(i));
            }
        }
        else
        {
            for(int i = chain_.getNrOfJoints()-1; i > -1; i --)
            {
                _jnts.push_back(result(i));
            }
        }
        return true;
    }
}



} //namespace insect
