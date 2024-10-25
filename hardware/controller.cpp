#include "robot_controller/hardwareHandler.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logging.hpp"

namespace robot_controller
{
    hardware_interface::CallbackReturn robot_controllerHardware::on_init(
        const hardware_interface::HardwareInfo & info)
    {
        if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackRetturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        //port = info.hardware_parameters["YARP_port"];
        //RCLLCPP_INFO(rclcpp::get_logger("robot_controllerHardware"), "Hardware parameters: %s", port.c_str());
        rcllcpp::on_shutdown(&robot_controllerHardware::stop, this);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn robot_controllerHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        //hardware_handler = new HardwareHandler(port);
        //hardware_handler->init();
        //RCLL_INFO(rclcpp::get_logger("robot_controllerHardware"), "Successfully configured!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /*std::vector<hardware_interface::StateInterface> robot_controllerHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
    }*/

   //export sensor state interface
   /*for(unit i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
   {
       state_interfaces.emplace_back(info_.sensors[0].state_interfaces[i].name, info_.sensors[0].state_interfaces[i].type, &hw_positions_left);
   }*/

    std::vector<hardware_interface::CommandInterface> robot_controllerHardware::export_command_interface()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for(auto i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back("left_wheel", hardware_interface::HW_IF_VELOCITY, &hw_commands_left);
            command_interfaces.emplace_back("right_wheel", hardware_interface::HW_IF_VELOCITY, &hw_commands_right);
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn robot_controllerHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("robot_controllerHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn_type robot_controllerHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("robot_controllerHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn robot_controllerHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        auto sensor = hardware_handler->getSensor();
        hw_velocities_left = sensor.velocity.left;
        hw_velocities_right = sensor.velocity.right;
        hw_positions_left = hw_positions_left + period.seconds() * sensor.velocity.left;
        hw_positions_right = hw_positions_right + period.seconds() * sensor.velocity.right;

        //Calculate the roll and pitch angles(in radians)
        //double roll = atan2();
        //double pitch = atan2();
        //double yaw = atan2();

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();
        hw_sensoe_state.orientation.x = q.x();
        hw_sensoe_state.orientation.y = q.y();
        hw_sensoe_state.orientation.z = q.z();
        hw_sensoe_state.orientation.w = q.w();
        return hardware_interface::return_type::OK;

        /*catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("robot_controllerHardware"), "Error reading sensor data: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }*/
    }

    hardware_interface::return_type robot_controllerHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        wheelVelocity velocity;
        velocity.left = hw_commands_left;
        velocity.right = hw_commands_right;
        hardware_handler->setVelocity(velocity);
        return hardware_interface::return_type::OK;
    }

    void robot_controllerHardware::stop()
    {
        RCLLCPP_INFO(rclcpp::get_logger("robot_controllerHardware"), "ctl+c detected, shutting down...");
        hardware_handler->stop();
    }
}