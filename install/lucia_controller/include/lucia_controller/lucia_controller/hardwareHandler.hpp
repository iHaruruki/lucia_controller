#ifndef ROBOT_CONTROLLER_SERIAL_HANDLER
#define ROBOT_CONTROLLER_SERIAL_HANDLER

//#include <libserial/SerialPort.h>

#include <string>
#include <thread>
#include <queue>

namespace robot_controller
{
    enum HardwareStatus
    {
        NOT_INITIALIZED,
        READY,
        EMERGENCY
    };

    struct Axis
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct IMUData
    {
        Axis accelerometer;
        Axis gyroscope;
        Axis magnetometer;
        double temperature = 0.0;
    };

    struct WheelVelocity
    {
        double left = 0.0;
        double right = 0.0;
    };

    struct SensorData
    {
        IMUData imu;
        WheelVelocity velocity;
    };

    struct ParsedData
    {
        std::string rawData;
        char type;
        int intData;
        double floatData;
    };

    struct ParsedSerialData
    {
        char command;
        int count = 0;
        std::string rawData;
        ParsedData data[20];
    };

    class HardwareHandler
    {
    public:
        HardwareHandler(std::string portname);
        ~HardwareHandler();
        // HardwareStatus getStatus();
        SensorData getSensor();
        void setVelocity(WheelVelocity velocity);
        bool init();
        void stop();

    private:
        bool running = true;
        HardwareStatus status = NOT_INITIALIZED;
        std::string portname;
        std::thread *serial_thread;
        LibSerial::SerialPort port;

        char buffer[256];
        int index = 0;

        bool serialCheck();
        ParsedSerialData parseSerialData();
    };

} // namespace

#endif // ROBOT_CONTROLLER_SERIAL_HANDLER