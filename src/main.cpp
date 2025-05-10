#include <rcutils/cmdline_parser.h>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "lucia_controller/diff_drive_controller.hpp"
#include "lucia_controller/lucia.hpp"

void help_print(){
    printf("For lucia_controller node : \n");
    printf("tlucia_controller_node [-i usb_port] [-h]\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    //printf("-i usb_port: Connected USB port with OpenCR.");
}

int main(int argc, char * argv[]){

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);  //バッファリングの制御

    if(rcutils_cli_option_exist(argv, argv + argc, "-h")){
        help_print();
        return 0;
    }

    rclcpp::init(argc, argv);

    std::string usb_port = "/dev/ttyACM0";
    char * cli_options;
    cli_options = rcutils_cli_get_option(argv, argv + argc, "-i");
    if (nullptr != cli_options) {
        usb_port = std::string(cli_options);
    }

    rclcpp::executors::SingleThreadedExecutor executor;

    auto lucia = std::make_shared<robotis::lucia::Lucia>(usb_port);
    auto diff_drive_controller =
        std::make_shared<robotics::lucia::DiffDriveController>(
            lucia->get_wheels()->separation,
            lucia->get_wheels()->radius);

    executor.add_node(lucia);
    executor.add_node(diff_drive_controller);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
