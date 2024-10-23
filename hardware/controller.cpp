#include "lucia_controller/controller.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_controller
{

    hardware_interface::CallbackReturn RobotControllerHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        port = (info_.hardware_parameters["serial_port"]);
        RCLCPP_INFO(rclcpp::get_logger("RobotControllerHardware"), "Hardware parameter - port: %s", port.c_str());
        rclcpp::on_shutdown(std::bind(&RobotControllerHardware::stop, this));
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotControllerHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        hardware_handler = new HardwareHandler(port);
        hardware_handler->init();
        RCLCPP_INFO(
            rclcpp::get_logger("RobotControllerHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RobotControllerHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            if (info_.joints[i].name.find("left") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_left));
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_left));
            }
            if (info_.joints[i].name.find("right") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_right));
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_right));
            }
        }

        // export sensor state interface
        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
        {
            // state_interfaces.emplace_back(hardware_interface::StateInterface(
            //     info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
            if (info_.sensors[0].state_interfaces[i].name.find("linear_acceleration.x") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_accel_x));
            }
            if (info_.sensors[0].state_interfaces[i].name.find("linear_acceleration.y") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_accel_y));
            }
            if (info_.sensors[0].state_interfaces[i].name.find("linear_acceleration.z") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_accel_z));
            }
            if (info_.sensors[0].state_interfaces[i].name.find("angular_velocity.x") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_gyro_x));
            }
            if (info_.sensors[0].state_interfaces[i].name.find("angular_velocity.y") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_gyro_y));
            }
            if (info_.sensors[0].state_interfaces[i].name.find("angular_velocity.z") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_gyro_z));
            }
            if (info_.sensors[0].state_interfaces[i].name.find("orientation.x") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_orientation_x));
            }
            if (info_.sensors[0].state_interfaces[i].name.find("orientation.y") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_orientation_y));
            }
            if (info_.sensors[0].state_interfaces[i].name.find("orientation.z") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_orientation_z));
            }
            if (info_.sensors[0].state_interfaces[i].name.find("orientation.w") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_orientation_w));
            }
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotControllerHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            if (info_.joints[i].name.find("left") != std::string::npos)
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_left));
            if (info_.joints[i].name.find("right") != std::string::npos)
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_right));
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn RobotControllerHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotControllerHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RobotControllerHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotControllerHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RobotControllerHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        try
        {
            auto sensor = hardware_handler->getSensor();
            hw_velocities_left = sensor.velocity.left;
            hw_velocities_right = sensor.velocity.right;
            hw_positions_left = hw_positions_left + period.seconds() * sensor.velocity.left;
            hw_positions_right = hw_positions_right + period.seconds() * sensor.velocity.right;

            hw_sensor_states_accel_x = sensor.imu.accelerometer.x / 1000 * 9.80665;
            hw_sensor_states_accel_y = sensor.imu.accelerometer.y / 1000 * 9.80665;
            hw_sensor_states_accel_z = sensor.imu.accelerometer.z / 1000 * 9.80665;
            // convert to radian/s from degree/s
            hw_sensor_states_gyro_x = sensor.imu.gyroscope.x * M_PI / 180;
            hw_sensor_states_gyro_y = sensor.imu.gyroscope.y * M_PI / 180;
            hw_sensor_states_gyro_z = sensor.imu.gyroscope.z * M_PI / 180;

            // Calculate the roll and pitch angles (in radians)
            double roll = atan2(hw_sensor_states_accel_y, hw_sensor_states_accel_z);
            double pitch = atan2(-hw_sensor_states_accel_x, sqrt(hw_sensor_states_accel_y * hw_sensor_states_accel_y + hw_sensor_states_accel_z * hw_sensor_states_accel_z));
            double yaw = atan2(sensor.imu.magnetometer.y / 1e6, sensor.imu.magnetometer.z / 1e6);

            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q.normalize();
            hw_sensor_states_orientation_x = q.x();
            hw_sensor_states_orientation_y = q.y();
            hw_sensor_states_orientation_z = q.z();
            hw_sensor_states_orientation_w = q.w();
            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotControllerHardware"), "Error writing to hardware: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    hardware_interface::return_type robot_controller::RobotControllerHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        try
        {
            hardware_handler->setVelocity({hw_commands_left, hw_commands_right});
            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotControllerHardware"), "Error writing to hardware: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    void RobotControllerHardware::stop()
    {
        //@TODO: Update implementation when there is an update to controller manager.
        // Implement workaround to handle ctrl-c termination.
        // Refer to issue - https://github.com/ros-controls/ros2_control/issues/472
        RCLCPP_INFO(rclcpp::get_logger("RobotControllerHardware"), "ctrl-c called, shutting down...");
        hardware_handler->stop();
    }

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    robot_controller::RobotControllerHardware, hardware_interface::SystemInterface)
