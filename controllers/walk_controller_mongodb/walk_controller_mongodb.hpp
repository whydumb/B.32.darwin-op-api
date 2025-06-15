// walk_controller_mongodb.hpp - 완전한 MongoDB 버전
#ifndef WALK_CONTROLLER_MONGODB_HPP
#define WALK_CONTROLLER_MONGODB_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

#ifdef USE_MONGODB
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/collection.hpp>
#include <bsoncxx/builder/stream/document.hpp>
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
  std::unique_ptr<mongocxx::instance> mMongoInstance;
  std::unique_ptr<mongocxx::client> mMongoClient;
  mongocxx::collection mCollection;
  bool mMongoConnected;
  int mMongoCheckCounter;
  
  // MongoDB 관련 메서드
  std::string getMongoAction();
  void executeMongoAction(const std::string& action);
#endif
};

#endif
