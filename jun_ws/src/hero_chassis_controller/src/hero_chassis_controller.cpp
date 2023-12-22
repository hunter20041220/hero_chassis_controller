#include "/home/hunter/jun_ws/src/hero_chassis_controller/include/hero_chassis_controller/hero_chassis_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/pid.h>


namespace hero_chassis_controller {
    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                       ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        front_left_joint_ =
                effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ =
                effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ =
                effort_joint_interface->getHandle("right_back_wheel_joint");

        p=controller_nh.param("/pid_param/p", 0.0);
        i=controller_nh.param("/pid_param/i", 0.0);
        d=controller_nh.param("/pid_param/d", 0.0);
        i_max=controller_nh.param("/pid_param/i_max", 0.0);
        i_min=controller_nh.param("/pid_param/i_min", 0.0);

        sub_command_ =controller_nh.subscribe("/cmd_vel", 1 , &HeroChassisController::setCommandCB,this);
        return true;
    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {

        control_toolbox::Pid pid;
        pid.initPid(p,i,d,i_max,i_min);

        real_vel_x=(front_left_joint_.getVelocity()+front_right_joint_.getVelocity()+
                back_left_joint_.getVelocity()+back_right_joint_.getVelocity())/4;
        real_vel_y=(-front_left_joint_.getVelocity()+front_right_joint_.getVelocity()+back_left_joint_.getVelocity()
                -back_right_joint_.getVelocity())/4;
        real_ang_z=((-front_left_joint_.getVelocity()+front_right_joint_.getVelocity()-back_left_joint_.getVelocity()
                +back_right_joint_.getVelocity())/4)/(0.2+0.2);
        //正运动学计算真实x方向和y方向角速度

        double error_x = cmd_vel_x - real_vel_x;
        double error_y = cmd_vel_y - real_vel_y;
        double error_z = cmd_ang_z - real_ang_z;
        //pid中误差计算

        cmd_wheel_tau_x = pid.computeCommand(error_x,period);
        cmd_wheel_tau_y = pid.computeCommand(error_y,period);
        cmd_wheel_tau_z = pid.computeCommand(error_z,period);

        front_left_joint_.setCommand((cmd_wheel_tau_x - cmd_wheel_tau_y - 0.4*cmd_wheel_tau_z)/0.07625);
        front_right_joint_.setCommand((cmd_wheel_tau_x + cmd_wheel_tau_y + 0.4*cmd_wheel_tau_z)/0.07625);
        back_left_joint_.setCommand((cmd_wheel_tau_x + cmd_wheel_tau_y - 0.4*cmd_wheel_tau_z)/0.07625);
        back_right_joint_.setCommand((cmd_wheel_tau_x - cmd_wheel_tau_y + 0.4*cmd_wheel_tau_z)/0.07625);
        //逆运动学公式
    }



    void HeroChassisController::setCommandCB(const geometry_msgs::Twist::ConstPtr& msg){

        cmd_vel_x = msg->linear.x;
        cmd_vel_y = msg->linear.y;
        cmd_ang_z = msg->linear.z;
    }
    


    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}











/*#include "/home/hunter/jun_ws/src/hero_chassis_controller/include/hero_chassis_controller/hero_chassis_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
//怎么把joint_改成四个轮子
namespace hero_chassis_controller {
    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &n)
    {
        // Get joint name from parameter server
        std::string joint_name;
        if (!n.getParam("joint", joint_name)) {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            return false;
        }

        // Get joint handle from hardware interface
        front_left_joint_ =
                effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ =
                effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ =
                effort_joint_interface->getHandle("right_back_wheel_joint");

        // Load PID Controller using gains set on parameter server
        if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
            return false;

        // Start realtime state publisher
        controller_state_publisher_.reset(
                new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
                        (n, "state", 1));

        // Start command subscriber
        sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &HeroChassisController::setCommandCB, this);
        return true;
    }

    void HeroChassisController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }

    void HeroChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }

    void HeroChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
    }

    void HeroChassisController::printDebug()
    {
        pid_controller_.printValues();
    }

    std::string HeroChassisController::getJointName()
    {
        return joint_.getName();
    }

// Set the joint velocity command
    void HeroChassisController::setCommand(double cmd)
    {
        command_ = cmd;
    }

// Return the current velocity command
    void HeroChassisController::getCommand(double& cmd)
    {
        cmd = command_;
    }

    void HeroChassisController::starting(const ros::Time& time)
    {
        command_ = 0.0;
        pid_controller_.reset();
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
    {
        double error = command_ - joint_.getVelocity();

        // Set the PID error and compute the PID command with nonuniform time
        // step size. The derivative error is computed from the change in the error
        // and the timestep dt.
        double commanded_effort = pid_controller_.computeCommand(error, period);

        joint_.setCommand(commanded_effort);

        if(loop_count_ % 10 == 0)
        {
            if(controller_state_publisher_ && controller_state_publisher_->trylock())
            {
                controller_state_publisher_->msg_.header.stamp = time;
                controller_state_publisher_->msg_.set_point = command_;
                controller_state_publisher_->msg_.process_value = joint_.getVelocity();
                controller_state_publisher_->msg_.error = error;
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = commanded_effort;

                double dummy;
                bool antiwindup;
                getGains(controller_state_publisher_->msg_.p,
                         controller_state_publisher_->msg_.i,
                         controller_state_publisher_->msg_.d,
                         controller_state_publisher_->msg_.i_clamp,
                         dummy,
                         antiwindup);
                controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_->unlockAndPublish();
            }
        }
        loop_count_++;
    }

    void HeroChassisController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
    {
        command_ = msg->data;
    }
    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}



#include <effort_controllers/joint_velocity_controller.h>
#include <pluginlib/class_list_macros.hpp>



PLUGINLIB_EXPORT_CLASS( effort_controllers::JointVelocityController, controller_interface::ControllerBase)
 */