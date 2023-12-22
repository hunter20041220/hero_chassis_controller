
#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

namespace hero_chassis_controller {

    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        HeroChassisController() = default;
        ~HeroChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void setCommandCB(const geometry_msgs::Twist::ConstPtr& msg);


        hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
    private:
        //int state_{};
        ros::Time last_change_;
        ros::Subscriber sub_command_;
        ros::Publisher pub_odom_;
        /*double front_left_wheel_vel, front_right_wheel_vel, back_left_wheel_vel, back_right_wheel_vel;*/
        double real_vel_x, real_vel_y, real_ang_z;
        double cmd_vel_x, cmd_vel_y, cmd_ang_z;
        double cmd_wheel_tau_x, cmd_wheel_tau_y, cmd_wheel_tau_z;
        double p, i, d, i_max, i_min;
        //double front_right_wheel_ang_vel, front_left_wheel_ang_vel, back_right_wheel_ang_vel, back_left_wheel_ang_vel;
    };
}// namespace simple_chassis_controller

#endif














/*#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#pragma once
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

namespace hero_chassis_controller {

    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        HeroChassisController() = default;
        ~HeroChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &n) override;

        void setCommand(double cmd);

        void getCommand(double & cmd);

        void starting(const ros::Time& time) override;

        void update(const ros::Time& time, const ros::Duration& period)override;

        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

        void printDebug();

        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);

        std::string getJointName();

        hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

        double command_;
    private:
        int state_{};
        int loop_count_;
        control_toolbox::Pid pid_controller_;

        std::unique_ptr<
                realtime_tools::RealtimePublisher<
                        control_msgs::JointControllerState> > controller_state_publisher_ ;

        ros::Subscriber sub_command_;


        void setCommandCB(const std_msgs::Float64ConstPtr& msg);
        ros::Time last_change_;
    };
}// namespace simple_chassis_controller

#endif //SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
 */

