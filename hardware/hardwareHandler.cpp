#include "robot_controller/hardwareHandler.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_controller
{
    HardwareHandler::HardwareHandler(std::string portname)
    {
        this->portname = portname;
    }

    bool HardwareHandler::init()
    {
        try
        {
            yarp::os::Network yarp;
            p_cmd.open("/remoteController/command:o");  //motor command
            p_enc.open("/remoteController/encoder:i");  //encoder reading

            // Connect to the sender port
            yarp::os::Network::connect("/remoteController/command:o","/vehicleDriver/remote:i");    //motor command
            yarp::os::Network::connect("/vehicleDriver/encoder:o", "/remoteController/encoder:i");  //encoder reading

            RCLL_INFO(rclcpp::get_logger("rclcpp"), "HardwareHandler initialized");

            //これでいいの？
            return true;
        }

        //ポートに接続できなかった場合のエラー処理を書く！！！！！！
        catch(!yarp::os::Network::connect("/remoteController/command:o", targetPort))
        {
            RCLL_ERROR(rclcpp::get_logger("rclcpp"), "Failed to connect to the sender port");
            return false;
        }

        //YARP 
        /*bool HardwareHandler::YarpCheck()
        {
            if (!yarp::os::Network::checkNetwork())
            {
                RCLL_ERROR(rclcpp::get_logger("rclcpp"), "YARP network is not found");
                return false;
            }
            return true;
        }*/
    }

    /*ParsedYarpData HardwareHandler::parseYarpData()
    {
        ParsedYarpData parsedData;
        yarp::os::Bottle *bottle = p_enc.read();
        if(bottle != NULL)
        {
            parsedData.command = bottle->get(0).asChar();
            parsedData.count = bottle->get(1).asInt();
            parsedData.rawData = bottle->toString();
            for(int i = 0; i < 4; i++)
            {
                parsedData.data[i].rawData = bottle->get(i+2).toString();
                parsedData.data[i].type = parsedData.data[i].rawData[0];
                if(parsedData.data[i].type == 'i')
                {
                    parsedData.data[i].intData = bottle->get(i+2).asInt();
                }
                else if(parsedData.data[i].type == 'd')
                {
                    parsedData.data[i].floatData = bottle->get(i+2).asDouble();
                }
            }
        }
        return parsedData;
    }*/

    /*SensorData HardwareHandler::getSensor()
    {
        SensorData sensorData;
        yarp::os::Bottle *bottle = p_enc.read();
        if(bottle != NULL)
        {
            sensorData.velocity.left = bottle->get(0).asDouble();
            sensorData.velocity.right = bottle->get(1).asDouble();
        }
        return sensorData;
    }*/

    void HardwareHandler::setVelocity(wheelVelocity velocity)
    {
        yarp::os::Bottle &bottle = p_cmd.prepare();
        bottle.clear();
        bottle.addDouble(velocity.left);
        bottle.addDouble(velocity.right);
        p_cmd.write();
    }

    void HardwareHandler::stop()
    {
        p_cmd.close();
        p_enc.close();
    }

    HardwareHandler::~HardwareHandler()
    {
        RCLL_INFO(rclcpp::get_logger("rclcpp"), "Closig YARP port");
        stop();
    }
} // namespace robot_controller