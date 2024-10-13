#ifndef SOLVER_H
#define SOLVER_H

#include <Eigen/Core>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kdl.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
namespace arx
{

class Arx5Solver
{

  public:
    Arx5Solver(std::string urdf_path, int joint_dof);
    Arx5Solver(std::string urdf_path, int joint_dof, std::string base_link, std::string eef_link,
               Eigen::Vector3d gravity_vector);
    ~Arx5Solver() = default;

    Eigen::VectorXd inverse_dynamics(Eigen::VectorXd joint_pos, Eigen::VectorXd joint_vel, Eigen::VectorXd joint_acc);
    std::tuple<bool, Eigen::VectorXd> inverse_kinematics(Eigen::Matrix<double, 6, 1> target_pose_6d,
                                                         Eigen::VectorXd current_joint_pos);
    Eigen::Matrix<double, 6, 1> forward_kinematics(Eigen::VectorXd joint_pos);

  private:
    // parameters for ik solver
    const double _EPS = 1E-5;
    const int _MAXITER = 500;
    const double _EPS_JOINTS = 1E-15;
    const int _JOINT_DOF;

    // These variables should be class members and will be used when solvers are called.
    KDL::Tree _tree;
    KDL::Chain _chain;
    KDL::Chain _chain_without_fixed_joints;

    std::shared_ptr<KDL::ChainIkSolverPos_LMA> _ik_solver;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> _fk_solver;
    std::shared_ptr<KDL::ChainIdSolver_RNE> _id_solver;
};
} // namespace arx

#endif