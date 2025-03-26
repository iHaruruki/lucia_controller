#include <rcutils/cmdline_parser.h>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

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

    auto lucia = std::make_shared<

    rclcpp::shutdown();
    return 0;
}
