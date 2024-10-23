#ifndef ROBOT_CONTROLLER
#define ROBOT_CONTROLLER

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "lucia_controller/hardwareHandler.hpp"

namespace robot_controller
{
    class RobotControllerHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RobotControllerHardware)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        void stop();

    private:
        HardwareHandler *hardware_handler;

        std::string port;

        double hw_commands_left = 0.0;
        double hw_commands_right = 0.0;

        double hw_positions_left = 0.0;
        double hw_positions_right = 0.0;
        double hw_velocities_left = 0.0;
        double hw_velocities_right = 0.0;

        double hw_sensor_states_accel_x = 0.0;
        double hw_sensor_states_accel_y = 0.0;
        double hw_sensor_states_accel_z = 0.0;
        double hw_sensor_states_gyro_x = 0.0;
        double hw_sensor_states_gyro_y = 0.0;
        double hw_sensor_states_gyro_z = 0.0;
        double hw_sensor_states_orientation_x = 0.0;
        double hw_sensor_states_orientation_y = 0.0;
        double hw_sensor_states_orientation_z = 0.0;
        double hw_sensor_states_orientation_w = 0.0;
    };

}

#endif