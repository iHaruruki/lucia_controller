//
//

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
    
    //struct IMUData

    struct wheelVelocity
    {
        double left = 0.0;
        double right = 0.0;
    };
    
    struct SensorData
    {
        wheelVelocity velocity;
    };
    
    struct ParsedData
    {
        std::string rawData;
        char type;
        int intData;
        double floatData;
    };

    //struct PorsedSerialData

    class HardwareHandler
    {
    public:
        HardwareHandler();
        ~HardwareHandler();
        //HardwareStatus getStatus();
        SensorData getSensor();
        void setVelocity(wheelVelocity velocity);
        bool init();
        void stop();

    private:
        bool running = true;
        HardwareStatus status = NOT_INITIALIZED;
        std::string portname;
        std::thread *serialThread;
        //LibSerial::SerialPort Port;

        char buffer[256];
        int index = 0;

        bool serialCheck();
        //ParsedSerialData parseSerialData();
    };
} // namespace robot_controller