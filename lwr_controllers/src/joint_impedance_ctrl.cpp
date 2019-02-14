#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <lwr_controllers/joint_impedance_ctrl.h>

namespace lwr_controllers {

GJointImpedanceController::GJointImpedanceController() {}

GJointImpedanceController::~GJointImpedanceController() {}

bool GJointImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());   
    q_des_.resize(kdl_chain_.getNrOfJoints());
    tau_des_.resize(kdl_chain_.getNrOfJoints());
    curr_q_.resize(kdl_chain_.getNrOfJoints());

    //jnt2jac_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    
    //J_kdl_ = KDL::Jacobian(kdl_chain_.getNrOfJoints());
 
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        tau_des_(i) = 0.0;//joint_handles_[i].getPosition();
        K_(i) = 0.0;
        D_(i) = 0.0;
        q_des_(i) = joint_set_point_handles_[i].getPosition();
        curr_q_(i) = joint_handles_[i].getPosition();
    }

    ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

    for (int i = 0; i < joint_handles_.size(); ++i) {
        if ( !nh_.getParam("stiffness_gains", K_(i) ) ) {
            ROS_WARN("Stiffness gain not set in yaml file, Using %f", K_(i));
        }
    }
    for (int i = 0; i < joint_handles_.size(); ++i) {
        if ( !nh_.getParam("damping_gains", D_(i)) ) {
            ROS_WARN("Damping gain not set in yaml file, Using %f", D_(i));
        }
    }

    typedef  const std_msgs::Float64MultiArray::ConstPtr& msg_type;
    sub_stiffness_ = nh_.subscribe<GJointImpedanceController, msg_type>("stiffness", 1, boost::bind(&GJointImpedanceController::setParam, this, _1, &K_, "K"));
    sub_damping_ = nh_.subscribe<GJointImpedanceController, msg_type>("damping", 1, boost::bind(&GJointImpedanceController::setParam, this, _1, &D_, "D"));
    sub_add_torque_ = nh_.subscribe<GJointImpedanceController, msg_type>("additional_torque", 1, boost::bind(&GJointImpedanceController::setParam, this, _1, &tau_des_, "AddTorque"));
    sub_posture_ = nh_.subscribe("command", 1, &GJointImpedanceController::command, this);

    ROS_INFO("WOOHOOO Real Joint Impedance");
    return true;


}

void GJointImpedanceController::starting(const ros::Time& time)
{
    cmd_mutex.lock();
    // Initializing stiffness, damping, ext_torque and set point values
    for (size_t i = 0; i < joint_handles_.size(); i++) {
        tau_des_(i) = 0.0;
        q_des_(i) = joint_handles_[i].getPosition();
        dq_(i) = joint_handles_[i].getVelocity();
        curr_q_(i) = joint_handles_[i].getPosition();

        stiffness_(i) = K_(i);
        damping_(i) = D_(i);
    }

    q_d_ = Eigen::Ref< Eigen::Matrix<double, 7, 1> > (q_des_.data);
    q_ = Eigen::Ref< Eigen::Matrix<double, 7, 1> > (curr_q_.data);

    cmd_mutex.unlock();


}

void GJointImpedanceController::update(const ros::Time& time, const ros::Duration& period)
{
    cmd_mutex.lock();
    //Compute control law. This controller sets all variables for the JointImpedance Interface from kuka

    // Initializing stiffness, damping, ext_torque and set point values
    for (size_t i = 0; i < joint_handles_.size(); i++) {
        curr_q_(i) = joint_handles_[i].getPosition();
        q_(i) = joint_handles_[i].getPosition();
        dq_(i) = joint_handles_[i].getVelocity();  

        stiffness_(i) = K_(i);
        damping_(i) = D_(i);  
    }

    

    // Calculate tau.
    // stiffness_ = Eigen::Ref< Eigen::Matrix<double, 7, 1> > (K_.data);
    // damping_ = Eigen::Ref< Eigen::Matrix<double, 7, 1> > (D_.data);
    error_ = q_d_ - q_;
    tau_d = stiffness_.cwiseProduct(error_) - damping_.cwiseProduct(dq_);


    
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        tau_des_(i) = tau_d(i);
        joint_handles_[i].setCommand(tau_d(i));
        joint_stiffness_handles_[i].setCommand(0.0);
        joint_damping_handles_[i].setCommand(0.0);
        joint_set_point_handles_[i].setCommand(q_(i));
    }
    cmd_mutex.unlock();

}


void GJointImpedanceController::command(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    if (msg->data.size() == 0) {
        ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
    else if ((int)msg->data.size() != joint_handles_.size()) {
        ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {
        
        ROS_INFO("GOT it!");
        cmd_mutex.lock();
        for (unsigned int j = 0; j < joint_handles_.size(); ++j){
            q_des_(j) = msg->data[j];
            q_d_(j) = msg->data[j];
        }
        cmd_mutex.unlock();
        ROS_INFO("Debug:");
        ROS_INFO("Qdes: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", q_des_(0),q_des_(1),q_des_(2),q_des_(3),q_des_(4),q_des_(5),q_des_(6));
        ROS_INFO("Taudes: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", tau_des_(0),tau_des_(1),tau_des_(2),tau_des_(3),tau_des_(4),tau_des_(5),tau_des_(6));
        ROS_INFO("Stiffness: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", stiffness_(0),stiffness_(1),stiffness_(2),stiffness_(3),stiffness_(4),stiffness_(5),stiffness_(6));
        ROS_INFO("Damping: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", damping_(0),damping_(1),damping_(2),damping_(3),damping_(4),damping_(5),damping_(6));
    }

}

void GJointImpedanceController::setParam(const std_msgs::Float64MultiArray_< std::allocator< void > >::ConstPtr& msg, KDL::JntArray* array, std::string s)
{
    if (msg->data.size() == joint_handles_.size())
    {
        cmd_mutex.lock();
        for (unsigned int i = 0; i < joint_handles_.size(); ++i)
        {
            (*array)(i) = msg->data[i];
        }
        cmd_mutex.unlock();
    }
    else
    {
        ROS_INFO("Num of Joint handles = %lu", joint_handles_.size());
    }

    ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

    ROS_INFO("New param %s: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", s.c_str(),
             (*array)(0), (*array)(1), (*array)(2), (*array)(3), (*array)(4), (*array)(5), (*array)(6));
}

} // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::GJointImpedanceController, controller_interface::ControllerBase)
