// Fixed walk_controller_mongodb.hpp
#ifndef WALK_MONGODB_HPP
#define WALK_MONGODB_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>
#include <string>
#include <vector>

// Windows socket includes
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    // Remove the pragma comment as it's not portable
    // Use linker flags instead: -lws2_32
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    typedef int SOCKET;
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket close
#endif

namespace managers {
    class RobotisOp2MotionManager;
    class RobotisOp2GaitManager;
}

namespace webots {
    class Motor;
    class PositionSensor;
    class LED;
    class Camera;
    class Accelerometer;
    class Gyro;
    class Keyboard;
    class Speaker;
}

struct SensorData {
    double accelerometer[3];
    double gyro[3];
    double motorPositions[NMOTORS];
    double timestamp;
    bool isWalking;
    std::string robotState;
};

struct CommandData {
    std::string command;
    std::vector<double> parameters;
    double timestamp;
};

class WalkMongoDB : public webots::Robot {
public:
    WalkMongoDB();
    virtual ~WalkMongoDB();
    void run();
    void checkIfFallen();

private:
    int mTimeStep;

    // Robot control methods
    void myStep();
    void wait(int ms);
    
    // Network methods
    void setupSocketServer();
    void handleClient(SOCKET clientSocket);  // Use SOCKET type for cross-platform compatibility
    void sendSensorData(SOCKET clientSocket);
    void processCommand(const CommandData& cmd);
    
    // Data serialization
    std::string serializeSensorData();
    CommandData deserializeCommand(const std::string& data);
    
    // Robot hardware
    webots::Motor *mMotors[NMOTORS];
    webots::PositionSensor *mPositionSensors[NMOTORS];
    webots::Accelerometer *mAccelerometer;
    webots::Gyro *mGyro;
    webots::Keyboard *mKeyboard;
    webots::Camera *mCamera;
    
    // Managers
    managers::RobotisOp2MotionManager *mMotionManager;
    managers::RobotisOp2GaitManager *mGaitManager;
    
    // Network
    SOCKET serverSocket;
    bool isServerRunning;
    int serverPort;
    
    // Robot state
    bool isWalking;
    SensorData currentSensorData;
};

#endif