#include <yarp::os::all.h>
#include <yarp::os::Network.h>
#include <yarp::os::Port.h>
#include <yarp::os::Bottle.h>

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

    struct PorsedYarpData
    {
        char command;
        int count;
        std::string rawDara;
        ParsedData data[4];
    }

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
        yarp::os::BufferedPort<yarp::os::Bottle> p_cmd; //motor command
        yarp::os::BufferedPort<yarp::os::Bottle> p_enc; //encoder reading

        char buffer[256];
        int index = 0;

        bool YarpCheck();
        ParsedYarplData parseYarpData();
    };
} // namespace robot_controller