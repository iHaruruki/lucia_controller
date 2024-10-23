#include "lucia_controller/hardwareHandler.hpp"
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
            port.Open(portname);
            port.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
            port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            port.SetParity(LibSerial::Parity::PARITY_NONE);
            port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            RCLCPP_INFO(rclcpp::get_logger("RobotControllerSerialHandler"), "Connected to portname %s", portname.c_str());
            // Starting robot
            port.Write("S\n");
            port.DrainWriteBuffer();
            while (!serialCheck())
                ;
            ParsedSerialData parsedSerialData = parseSerialData();
            if (parsedSerialData.command == 'S')
            {
                RCLCPP_INFO(rclcpp::get_logger("RobotControllerSerialHandler"), "Robot started");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("RobotControllerSerialHandler"), "Error starting robot");
                return false;
            }
            return true;
        }
        catch (const LibSerial::OpenFailed &)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotControllerSerialHandler"), "Failed to connect to %s", portname.c_str());
            return false;
        }
    }

    bool HardwareHandler::serialCheck()
    {
        size_t ms_timeout = 100;
        char data_byte;
        while (port.IsDataAvailable())
        {
            try
            {
                port.ReadByte(data_byte, ms_timeout);
                if (data_byte == '\n')
                {
                    buffer[index] = '\0';
                    index = 0;
                    return true;
                }
                else
                {
                    buffer[index++] = data_byte;
                }
            }
            catch (const LibSerial::ReadTimeout &)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RobotControllerSerialHandler"), "Port %s read timeout", portname.c_str());
                running = false;
                return false;
            }
        }
        return false;
    }

    ParsedSerialData HardwareHandler::parseSerialData()
    {
        char dataType[20];
        char rawData[256];
        ParsedSerialData parsedSerialData;
        parsedSerialData.command = buffer[0];
        parsedSerialData.count = 0;
        int i = 1;
        int dataTypeIndex = 0;
        int rawDataIndex = 0;
        bool gettingDataType = true;
        parsedSerialData.rawData = std::string(buffer);
        while (buffer[i] != '\0')
        {
            if (gettingDataType)
            {
                if (buffer[i] == ':')
                {
                    gettingDataType = false;
                    dataTypeIndex = 0;
                }
                else
                    dataType[dataTypeIndex++] = buffer[i];
            }
            else
            {
                if (buffer[i] == ';')
                {
                    parsedSerialData.data[parsedSerialData.count].type = dataType[parsedSerialData.count];
                    parsedSerialData.data[parsedSerialData.count].rawData = std::string(rawData);
                    if (dataType[parsedSerialData.count] == 'i')
                        parsedSerialData.data[parsedSerialData.count].intData = atoi(rawData);
                    else if (dataType[parsedSerialData.count] == 'f')
                        parsedSerialData.data[parsedSerialData.count].floatData = atof(rawData);
                    rawDataIndex = 0;
                    parsedSerialData.count++;
                }
                else
                    rawData[rawDataIndex++] = buffer[i];
            }
            i++;
        }
        return parsedSerialData;
    }

    SensorData HardwareHandler::getSensor()
    {
        port.Write("d\n");
        port.DrainWriteBuffer();
        while (!serialCheck())
            ;
        ParsedSerialData parsedSerialData = parseSerialData();
        if (parsedSerialData.command == 'd')
        {
            WheelVelocity measured_velocity;
            measured_velocity.left = parsedSerialData.data[0].floatData;
            measured_velocity.right = parsedSerialData.data[1].floatData;
            IMUData imu_data;
            imu_data.accelerometer.x = parsedSerialData.data[2].floatData;
            imu_data.accelerometer.y = parsedSerialData.data[3].floatData;
            imu_data.accelerometer.z = parsedSerialData.data[4].floatData;
            imu_data.gyroscope.x = parsedSerialData.data[5].floatData;
            imu_data.gyroscope.y = parsedSerialData.data[6].floatData;
            imu_data.gyroscope.z = parsedSerialData.data[7].floatData;
            imu_data.magnetometer.x = parsedSerialData.data[8].floatData;
            imu_data.magnetometer.y = parsedSerialData.data[9].floatData;
            imu_data.magnetometer.z = parsedSerialData.data[10].floatData;
            imu_data.temperature = parsedSerialData.data[11].floatData;
            return {imu_data, measured_velocity};
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotControllerSerialHandler"), "Error getting velocity");
            WheelVelocity measured_velocity;
            IMUData imu_data;
            return {imu_data, measured_velocity};
        }
    }

    void HardwareHandler::setVelocity(WheelVelocity velocity)
    {
        port.Write("mff:");
        std::string left = std::to_string(velocity.left);
        std::string right = std::to_string(velocity.right);
        port.Write(left.substr(0, left.find(".") + 3));
        port.WriteByte(';');
        port.Write(right.substr(0, right.find(".") + 3));
        port.Write(";\n");
        port.DrainWriteBuffer();
    }

    void HardwareHandler::stop()
    {
        port.Write("s\n");
        port.DrainWriteBuffer();
        RCLCPP_INFO(rclcpp::get_logger("RobotControllerSerialHandler"), "Robot stopped");
    }

    HardwareHandler::~HardwareHandler()
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotControllerSerialHandler"), "Closing serial portname %s", portname.c_str());
        stop();
    }
}