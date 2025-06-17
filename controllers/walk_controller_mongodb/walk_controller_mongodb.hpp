// walk_controller_mongodb.hpp - Fixed MongoDB version
#ifndef WALK_CONTROLLER_MONGODB_HPP
#define WALK_CONTROLLER_MONGODB_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

#ifdef USE_MONGODB
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/uri.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <bsoncxx/view_or_value.hpp>
#include <memory>
#include <string>
#endif

namespace managers {
class RobotisOp2MotionManager;
class RobotisOp2GaitManager;
}

namespace webots {
class Motor;
class PositionSensor;
class LED;
class Accelerometer;
}

class Walk : public webots::Robot {
public:
  Walk();
  virtual ~Walk();
  void run();
  void checkIfFallen();

private:
  int mTimeStep;

  void myStep();
  void wait(int ms);

  webots::Motor *mMotors[NMOTORS];
  webots::PositionSensor *mPositionSensors[NMOTORS];
  webots::Accelerometer *mAccelerometer;

  managers::RobotisOp2MotionManager *mMotionManager;
  managers::RobotisOp2GaitManager *mGaitManager;

#ifdef USE_MONGODB
  // MongoDB instance and connection
  std::unique_ptr<mongocxx::instance> mMongoInstance;
  std::unique_ptr<mongocxx::client> mMongoClient;
  mongocxx::collection mCollection;
  
  // Connection status and configuration
  bool mMongoConnected;
  int mMongoCheckCounter;
  std::string mLastAction;
  int mReconnectAttempts;
  static const int MAX_RECONNECT_ATTEMPTS = 5;
  
  // Configuration values - ADDED MISSING DECLARATIONS
  std::string mDbName;
  std::string mCollectionName;
  std::string mReplayName;
  std::string mMongoUri;
  int mPollInterval;
  
  // MongoDB related methods
  void loadMongoConfig();
  void initializeMongoDB();
  void testMongoConnection();
  bool reconnectMongoDB();
  std::string getMongoAction();
  void executeMongoAction(const std::string& action);
#endif
};

#endif
