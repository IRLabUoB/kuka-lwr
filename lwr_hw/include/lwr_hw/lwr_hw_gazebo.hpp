#ifndef LWR_HW____LWR_HW_SIM_H
#define LWR_HW____LWR_HW_SIM_H

// ROS
#include <angles/angles.h>
// #include <pluginlib/class_list_macros.h>

// Gazebo hook
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

#include "lwr_hw/eigen.hpp"

namespace lwr_hw {


class LWRHWGazebo : public LWRHW
{
public:

  LWRHWGazebo() : LWRHW(), ts_(3.0), lastT_(0.0) {}
  ~LWRHWGazebo() {}

  void setParentModel(gazebo::physics::ModelPtr parent_model){parent_model_ = parent_model; parent_set_ = true;};

  // Init, read, and write, with Gazebo hooks
  bool init()
  {
    if( !(parent_set_) )
    {
      std::cout << "Did you forget to set the parent model?" << std::endl << "You must do that before init()" << std::endl << "Exiting..." << std::endl;
      return false;
    }

    gazebo::physics::JointPtr joint;
    for(int j=0; j < n_joints_; j++)
    {
      joint = parent_model_->GetJoint(joint_names_[j]);
      if (!joint)
      {
        std::cout << "This robot has a joint named \"" << joint_names_[j]
          << "\" which is not in the gazebo model." << std::endl;
        return false;
      }
      sim_joints_.push_back(joint);
    }

    return true;
  }

  void read(ros::Time time, ros::Duration period)
  {
    for(int j=0; j < n_joints_; ++j)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                              sim_joints_[j]->GetAngle(0).Radian());
      joint_position_kdl_(j) = joint_position_[j];
      // derivate velocity as in the real hardware instead of reading it from simulation
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j] - joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
      joint_velocity_kdl_(j) = joint_velocity_[j];
      joint_effort_[j] = sim_joints_[j]->GetForce((int)(0));
      joint_stiffness_[j] = joint_stiffness_command_[j];
    }

    fk_pos_solver_->JntToCart(joint_position_kdl_,ee_frame_);

    ee_position_ = eigen_utils::toEigen(ee_frame_.p);
    ee_orientation_ = eigen_utils::toEigen(ee_frame_.M);
    ee_transform_ = eigen_utils::toEigen(ee_frame_);
    jnt2jac_->JntToJac(joint_position_kdl_, J_kdl_);

    f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);
    f_dyn_solver_->JntToCoriolis(joint_position_kdl_, joint_velocity_kdl_, coriolis_effort_);
    
  }

  void write(ros::Time time, ros::Duration period)
  {
    enforceLimits(period);

    Eigen::Matrix<double, 3, 3> rotation;
    
    

    rotation(0,0) = cart_pos_command_[0]; rotation(1,0) = cart_pos_command_[4]; rotation(2,0) = cart_pos_command_[8];
    rotation(0,1) = cart_pos_command_[1]; rotation(1,1) = cart_pos_command_[5]; rotation(2,1) = cart_pos_command_[9];
    rotation(0,2) = cart_pos_command_[2]; rotation(1,2) = cart_pos_command_[6]; rotation(2,2) = cart_pos_command_[10];
    Eigen::Quaterniond orientation_d(rotation);
    Eigen::Vector3d position_d(cart_pos_command_[3],cart_pos_command_[7],cart_pos_command_[11]);

    Eigen::Map<const Eigen::Matrix<double, 6, 1> > wrench_ext(cart_wrench_command_.data());

    Eigen::Ref< Eigen::Matrix<double, 7, 1> > coriolis(coriolis_effort_.data);
    Eigen::Ref< Eigen::Matrix<double, 7, 1> > gravity(gravity_effort_.data);
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(joint_velocity_.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 1> > stiffness(cart_stiff_command_.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 1> > damping(cart_damp_command_.data());
    
    //cart_damp_command_
    // /Eigen::Map<Eigen::Matrix<double, 7, 1> > gravity(gravity_effort_.data);

    Eigen::Matrix<double,6,7> jacobian = J_kdl_.data.cast<double>();

    // Position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << ee_position_ - position_d;

    // Orientation error
    Eigen::Quaterniond error_quaternion(ee_orientation_.inverse() * orientation_d);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -ee_transform_.linear() * error.tail(3);

    Eigen::VectorXd tau_cmd(7), tau_d(7), tau_dyn(7);
    

    switch (getControlStrategy())
    {

      case JOINT_POSITION:
        for(int j=0; j < n_joints_; j++)
        {
          // according to the gazebo_ros_control plugin, this must *not* be called if SetForce is going to be called
          // but should be called when SetPostion is going to be called
          // so enable this when I find the SetMaxForce reset.
          // sim_joints_[j]->SetMaxForce(0, joint_effort_limits_[j]);
#if GAZEBO_MAJOR_VERSION >= 4
          sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
#else
          sim_joints_[j]->SetAngle(0, joint_position_command_[j]);
#endif
        }
        break;

      case CARTESIAN_IMPEDANCE:
        
        // Spring damper system with damping ratio=1
        tau_dyn = coriolis + gravity;
        tau_cmd << jacobian.transpose() * (-stiffness.cwiseProduct(error) - damping.cwiseProduct(jacobian * dq) + wrench_ext);
        tau_d << tau_cmd + tau_dyn;
        
        for(int j=0; j < n_joints_; j++)
        {
          sim_joints_[j]->SetForce(0, tau_d(j));
        }

        if(time-lastT_ < ts_)
            break;
        lastT_ = time;
        ROS_WARN("CARTESIAN IMPEDANCE IMPLEMENTATION IN GAZEBO, PRINTING THE COMMANDED VALUES:");
        std::cout << "Notice that this printing is done only every " << ts_.toSec() << " seconds: to change this, change ts_ in lwr_hw_gazebo.hpp..." << std::endl;
        std::cout << "cart_pos_command_ = | ";
        for(int i=0; i < 12; ++i)
            std::cout << cart_pos_command_[i] << " | ";
        std::cout << std::endl << "cart_stiff_command_ = | ";
        for(int i=0; i < 6; i++)
            std::cout << cart_stiff_command_[i] << " | ";
        std::cout << std::endl << "cart_damp_command_ = | ";
        for(int i=0; i < 6; i++)
            std::cout << cart_damp_command_[i] << " | ";
        std::cout << std::endl << "cart_wrench_command_ = | ";
        for(int i=0; i < 6; i++)
            std::cout << cart_wrench_command_[i] << " | ";
        std::cout << std::endl << "Here, the call to doCartesianImpedanceControl() is done" << std::endl;
        break;

      case JOINT_IMPEDANCE:
        // compute the gracity term
        
        for(int j=0; j < n_joints_; j++)
        {
          // replicate the joint impedance control strategy
          // tau = k (q_FRI - q_msr) + tau_FRI + D(q_msr) + f_dyn(q_msr)
          const double stiffness_effort = joint_stiffness_command_[j]*( joint_position_command_[j] - joint_position_[j] );//0.0;//10.0*( joint_position_command_[j] - joint_position_[j] ); // 
          double damping_effort = joint_damping_command_[j]*( joint_velocity_[j] );
          double f_dyn = gravity_effort_(j) + coriolis_effort_(j);
          const double effort = stiffness_effort + joint_effort_command_[j] + damping_effort + f_dyn;
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case GRAVITY_COMPENSATION:
        ROS_WARN("CARTESIAN IMPEDANCE NOT IMPLEMENTED");
        break;
    }
  }

private:

  // Gazebo stuff
  std::vector<gazebo::physics::JointPtr> sim_joints_;
  gazebo::physics::ModelPtr parent_model_;
  bool parent_set_ = false;
  ros::Duration ts_;
  ros::Time lastT_;

};

}

#endif