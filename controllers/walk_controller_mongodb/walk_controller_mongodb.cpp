// walk_controller_mongodb.cpp - Fixed MongoDB version
#include "walk_controller_mongodb.hpp"
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#ifdef USE_MONGODB
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/uri.hpp>
#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/types.hpp>
#include <bsoncxx/view_or_value.hpp>
#endif

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace webots;
using namespace managers;
using namespace std;

#ifdef USE_MONGODB
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
#endif

static const char *motorNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
  "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
  "AnkleL", "FootR", "FootL", "Neck", "Head"
};

Walk::Walk() : Robot() {
  mTimeStep = static_cast<int>(getBasicTimeStep());

  // Initialize LEDs with error checking
  LED* headLed = getLED("HeadLed");
  LED* eyeLed = getLED("EyeLed");
  if (headLed) headLed->set(0xFF0000);
  if (eyeLed) eyeLed->set(0x00FF00);
  
  mAccelerometer = getAccelerometer("Accelerometer");
  if (mAccelerometer) {
    mAccelerometer->enable(mTimeStep);
  }

  // Initialize motors and position sensors
  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    if (mPositionSensors[i]) {
      mPositionSensors[i]->enable(mTimeStep);
    }
  }

  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");

#ifdef USE_MONGODB
  // Initialize MongoDB variables with default values
  mPollInterval = 500; // Default 500ms
  mMongoConnected = false;
  mMongoCheckCounter = 0;
  mLastAction = "idle";
  mReconnectAttempts = 0;
  
  // Load MongoDB configuration from config.ini
  loadMongoConfig();
  initializeMongoDB();
#endif
}

Walk::~Walk() {
  if (mMotionManager) {
    delete mMotionManager;
  }
  if (mGaitManager) {
    delete mGaitManager;
  }
}

#ifdef USE_MONGODB
void Walk::loadMongoConfig() {
  // Set default values
  mDbName = "movement_tracker";
  mCollectionName = "movementtracker";
  mReplayName = "default_replay";
  mPollInterval = 500;
  mMongoUri = "mongodb://localhost:27017";
  
  ifstream configFile("config.ini");
  if (!configFile.is_open()) {
    cout << "[Config] config.ini file not found. Using default values" << endl;
    return;
  }
  
  string line;
  while (getline(configFile, line)) {
    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') continue;
    
    if (line.find("replay_name") != string::npos) {
      size_t pos = line.find("=");
      if (pos != string::npos) {
        mReplayName = line.substr(pos + 1);
        // Remove whitespace
        mReplayName.erase(0, mReplayName.find_first_not_of(" \t"));
        mReplayName.erase(mReplayName.find_last_not_of(" \t") + 1);
      }
    }
    else if (line.find("poll_interval") != string::npos) {
      size_t pos = line.find("=");
      if (pos != string::npos) {
        string value = line.substr(pos + 1);
        try {
          mPollInterval = stoi(value);
          if (mPollInterval < 100) mPollInterval = 100; // Minimum 100ms
        } catch (const exception& e) {
          cout << "[Config] Invalid poll_interval value, using default 500ms" << endl;
          mPollInterval = 500;
        }
      }
    }
    else if (line.find("mongo_uri") != string::npos) {
      size_t pos = line.find("=");
      if (pos != string::npos) {
        mMongoUri = line.substr(pos + 1);
        // Remove whitespace
        mMongoUri.erase(0, mMongoUri.find_first_not_of(" \t"));
        mMongoUri.erase(mMongoUri.find_last_not_of(" \t") + 1);
      }
    }
    else if (line.find("database_name") != string::npos) {
      size_t pos = line.find("=");
      if (pos != string::npos) {
        mDbName = line.substr(pos + 1);
        mDbName.erase(0, mDbName.find_first_not_of(" \t"));
        mDbName.erase(mDbName.find_last_not_of(" \t") + 1);
      }
    }
    else if (line.find("collection_name") != string::npos) {
      size_t pos = line.find("=");
      if (pos != string::npos) {
        mCollectionName = line.substr(pos + 1);
        mCollectionName.erase(0, mCollectionName.find_first_not_of(" \t"));
        mCollectionName.erase(mCollectionName.find_last_not_of(" \t") + 1);
      }
    }
  }
  
  cout << "[Config] Replay Name: " << mReplayName << ", Poll Interval: " << mPollInterval << "ms" << endl;
  cout << "[Config] MongoDB URI: " << mMongoUri << endl;
  cout << "[Config] Database: " << mDbName << ", Collection: " << mCollectionName << endl;
}

void Walk::initializeMongoDB() {
  try {
    // Initialize MongoDB instance (singleton)
    static mongocxx::instance instance{};
    
    mMongoClient = make_unique<mongocxx::client>(mongocxx::uri{mMongoUri});
    mCollection = (*mMongoClient)[mDbName][mCollectionName];
    mMongoConnected = true;
    mMongoCheckCounter = 0;
    mLastAction = "idle";
    mReconnectAttempts = 0;
    
    // Test connection
    testMongoConnection();
    
    cout << "[MongoDB] Connection successful - " << mDbName << "." << mCollectionName << endl;
  } catch (const exception& e) {
    mMongoConnected = false;
    cout << "[MongoDB] Connection failed: " << e.what() << endl;
  }
}

void Walk::testMongoConnection() {
  try {
    // Simple ping test
    mongocxx::database admin = (*mMongoClient)["admin"];
    auto result = admin.run_command(document{} << "ping" << 1 << finalize);
    cout << "[MongoDB] Connection test successful" << endl;
  } catch (const exception& e) {
    cout << "[MongoDB] Connection test failed: " << e.what() << endl;
    throw;
  }
}

bool Walk::reconnectMongoDB() {
  if (mReconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
    cout << "[MongoDB] Maximum reconnection attempts exceeded" << endl;
    return false;
  }
  
  cout << "[MongoDB] Reconnection attempt " << (mReconnectAttempts + 1) << "/" << MAX_RECONNECT_ATTEMPTS << endl;
  mReconnectAttempts++;
  
  try {
    mMongoClient = make_unique<mongocxx::client>(mongocxx::uri{mMongoUri});
    mCollection = (*mMongoClient)[mDbName][mCollectionName];
    testMongoConnection();
    
    mMongoConnected = true;
    mReconnectAttempts = 0;
    cout << "[MongoDB] Reconnection successful" << endl;
    return true;
  } catch (const exception& e) {
    cout << "[MongoDB] Reconnection failed: " << e.what() << endl;
    return false;
  }
}

string Walk::getMongoAction() {
  if (!mMongoConnected) {
    if (!reconnectMongoDB()) {
      return mLastAction; // Keep last successful action
    }
  }

  try {
    // Filter by replay_name
    auto filter = document{} << "replay_name" << mReplayName << finalize;
    
    auto maybe_result = mCollection.find_one(filter.view());
    
    if (maybe_result) {
      auto doc = maybe_result.value();
      auto action_element = doc["current_action"];
      
      if (action_element && action_element.type() == bsoncxx::type::k_string) {
        string action = string(action_element.get_string().value);
        
        // Log only when action changes
        if (action != mLastAction) {
          cout << "[MongoDB] Replay '" << mReplayName << "' new action: " << action << endl;
          mLastAction = action;
        }
        
        mReconnectAttempts = 0; // Reset reconnection counter on success
        return action;
      } else {
        cout << "[MongoDB] current_action field missing or wrong type" << endl;
        return mLastAction;
      }
    } else {
      cout << "[MongoDB] No document found with replay_name '" << mReplayName << "'" << endl;
      return mLastAction;
    }
    
  } catch (const exception& e) {
    cout << "[MongoDB] Query error: " << e.what() << endl;
    mMongoConnected = false;
    return mLastAction;
  }
}

void Walk::executeMongoAction(const string& action) {
  if (!mGaitManager) return;
  
  if (action == "forward") {
    mGaitManager->setXAmplitude(1.0);
    mGaitManager->setAAmplitude(0.0);
  }
  else if (action == "backward") {
    mGaitManager->setXAmplitude(-1.0);
    mGaitManager->setAAmplitude(0.0);
  }
  else if (action == "left") {
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.5);
  }
  else if (action == "right") {
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(-0.5);
  }
  else { // "idle" or other
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
  }
}
#endif

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void Walk::run() {
  cout << "=======================================" << endl;
  cout << "MongoDB Integrated ROBOTIS OP2 Walk Controller" << endl;
  cout << "=======================================" << endl;
  
#ifdef USE_MONGODB
  cout << "Reading current_action from MongoDB with replay_name='" << mReplayName << "'" << endl;
  cout << "for automatic control." << endl;
  cout << "Polling interval: " << mPollInterval << "ms" << endl;
#else
  cout << "MongoDB disabled - staying in idle state" << endl;
#endif
  cout << "=======================================" << endl;

  myStep();
  
  cout << "[Initialize] Moving to basic position..." << endl;
  if (mMotionManager) {
    mMotionManager->playPage(9);
    wait(200);
  }

#ifdef USE_MONGODB
  if (mMongoConnected && mGaitManager) {
    mGaitManager->start();
    cout << "[Start] Walking mode activated" << endl;
    cout << "[Waiting] Waiting for commands from MongoDB..." << endl;
  } else {
    cout << "[Error] MongoDB connection failed - staying in idle state" << endl;
  }
#endif

  // Calculate polling interval (in mTimeStep units)
  int pollSteps = (mTimeStep > 0) ? (mPollInterval / mTimeStep) : 62; // Default ~500ms at 8ms timestep
  if (pollSteps < 1) pollSteps = 1;

  cout << "[Info] Poll steps: " << pollSteps << " (TimeStep: " << mTimeStep << "ms)" << endl;

  while (true) {
    checkIfFallen();

#ifdef USE_MONGODB
    if (mMongoConnected || mReconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
      mMongoCheckCounter++;
      if (mMongoCheckCounter >= pollSteps) {
        string mongoAction = getMongoAction();
        executeMongoAction(mongoAction);
        mMongoCheckCounter = 0;
      }
    }
#endif

    if (mGaitManager) {
      mGaitManager->step(mTimeStep);
    }
    myStep();
  }
}

void Walk::checkIfFallen() {
  if (!mAccelerometer) return;
  
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;

  const double *acc = mAccelerometer->getValues();
  if (acc && acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;

  if (acc && acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;

  if (fup > acc_step && mMotionManager) {
    cout << "[Fall Detection] Fell forward - recovering..." << endl;
    mMotionManager->playPage(10); // f_up
    mMotionManager->playPage(9);  // init position
    fup = 0;
  }
  else if (fdown > acc_step && mMotionManager) {
    cout << "[Fall Detection] Fell backward - recovering..." << endl;
    mMotionManager->playPage(11); // b_up
    mMotionManager->playPage(9);  // init position
    fdown = 0;
  }
}
