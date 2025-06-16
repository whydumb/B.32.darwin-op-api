// walk_controller_mongodb.cpp - 개선된 MongoDB 버전
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
  mTimeStep = getBasicTimeStep();

  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);
  
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
  }

  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");

#ifdef USE_MONGODB
  // config.ini에서 MongoDB 설정 읽기
  loadMongoConfig();
  initializeMongoDB();
#endif
}

Walk::~Walk() {
  delete mMotionManager;
  delete mGaitManager;
}

#ifdef USE_MONGODB
void Walk::loadMongoConfig() {
  // 기본값 설정
  mDbName = "movement_tracker";
  mCollectionName = "movementtracker";
  mReplayName = "your_database_name";
  mPollInterval = 500;
  mMongoUri = "mongodb://localhost:27017";
  
  ifstream configFile("config.ini");
  if (!configFile.is_open()) {
    cout << "[Config] config.ini 파일을 찾을 수 없습니다. 기본값 사용" << endl;
    return;
  }
  
  string line;
  while (getline(configFile, line)) {
    if (line.find("replay_name") != string::npos) {
      size_t pos = line.find("=");
      if (pos != string::npos) {
        mReplayName = line.substr(pos + 1);
        // 공백 제거
        mReplayName.erase(0, mReplayName.find_first_not_of(" \t"));
        mReplayName.erase(mReplayName.find_last_not_of(" \t") + 1);
      }
    }
    else if (line.find("poll_interval") != string::npos) {
      size_t pos = line.find("=");
      if (pos != string::npos) {
        string value = line.substr(pos + 1);
        mPollInterval = stoi(value);
      }
    }
  }
  
  cout << "[Config] Replay Name: " << mReplayName << ", Poll Interval: " << mPollInterval << "ms" << endl;
}

void Walk::initializeMongoDB() {
  try {
    mMongoInstance = make_unique<mongocxx::instance>();
    mMongoClient = make_unique<mongocxx::client>(mongocxx::uri{mMongoUri});
    mCollection = (*mMongoClient)[mDbName][mCollectionName];
    mMongoConnected = true;
    mMongoCheckCounter = 0;
    mLastAction = "idle";
    mReconnectAttempts = 0;
    
    // 연결 테스트
    testMongoConnection();
    
    cout << "[MongoDB] 연결 성공 - " << mDbName << "." << mCollectionName << endl;
  } catch (const exception& e) {
    mMongoConnected = false;
    cout << "[MongoDB] 연결 실패: " << e.what() << endl;
  }
}

void Walk::testMongoConnection() {
  try {
    // 간단한 ping 테스트
    auto admin = (*mMongoClient)["admin"];
    auto result = admin.run_command(document{} << "ping" << 1 << finalize);
    cout << "[MongoDB] 연결 테스트 성공" << endl;
  } catch (const exception& e) {
    cout << "[MongoDB] 연결 테스트 실패: " << e.what() << endl;
    throw;
  }
}

bool Walk::reconnectMongoDB() {
  if (mReconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
    cout << "[MongoDB] 최대 재연결 시도 횟수 초과" << endl;
    return false;
  }
  
  cout << "[MongoDB] 재연결 시도 " << (mReconnectAttempts + 1) << "/" << MAX_RECONNECT_ATTEMPTS << endl;
  mReconnectAttempts++;
  
  try {
    mMongoClient = make_unique<mongocxx::client>(mongocxx::uri{mMongoUri});
    mCollection = (*mMongoClient)[mDbName][mCollectionName];
    testMongoConnection();
    
    mMongoConnected = true;
    mReconnectAttempts = 0;
    cout << "[MongoDB] 재연결 성공" << endl;
    return true;
  } catch (const exception& e) {
    cout << "[MongoDB] 재연결 실패: " << e.what() << endl;
    return false;
  }
}

string Walk::getMongoAction() {
  if (!mMongoConnected) {
    if (!reconnectMongoDB()) {
      return mLastAction; // 마지막 성공한 액션 유지
    }
  }

  try {
    // replay_name으로 필터링
    auto filter = document{} << "replay_name" << mReplayName << finalize;
    
    auto maybe_result = mCollection.find_one(filter.view());
    
    if (maybe_result) {
      auto doc = maybe_result.value();
      auto action_element = doc["current_action"];
      
      if (action_element && action_element.type() == bsoncxx::type::k_utf8) {
        string action = action_element.get_utf8().value.to_string();
        
        // 액션이 변경되었을 때만 로그 출력
        if (action != mLastAction) {
          cout << "[MongoDB] Replay '" << mReplayName << "' 새 액션: " << action << endl;
          mLastAction = action;
        }
        
        mReconnectAttempts = 0; // 성공시 재연결 카운터 리셋
        return action;
      } else {
        cout << "[MongoDB] current_action 필드가 없거나 잘못된 타입입니다." << endl;
        return mLastAction;
      }
    } else {
      cout << "[MongoDB] replay_name '" << mReplayName << "'인 문서를 찾을 수 없습니다." << endl;
      return mLastAction;
    }
    
  } catch (const exception& e) {
    cout << "[MongoDB] 쿼리 오류: " << e.what() << endl;
    mMongoConnected = false;
    return mLastAction;
  }
}

void Walk::executeMongoAction(const string& action) {
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
  else { // "idle" 또는 기타
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
  cout << "MongoDB 통합 ROBOTIS OP2 Walk Controller" << endl;
  cout << "=======================================" << endl;
  
#ifdef USE_MONGODB
  cout << "MongoDB에서 replay_name='" << mReplayName << "'의" << endl;
  cout << "current_action 값을 읽어서 자동 제어합니다." << endl;
  cout << "폴링 간격: " << mPollInterval << "ms" << endl;
#else
  cout << "MongoDB 비활성화 - 정지 상태" << endl;
#endif
  cout << "=======================================" << endl;

  myStep();
  
  cout << "[초기화] 기본 자세로 이동중..." << endl;
  mMotionManager->playPage(9);
  wait(200);

#ifdef USE_MONGODB
  if (mMongoConnected) {
    mGaitManager->start();
    cout << "[시작] 걷기 모드 활성화" << endl;
    cout << "[대기] MongoDB에서 명령 대기중..." << endl;
  } else {
    cout << "[오류] MongoDB 연결 실패 - 정지 상태" << endl;
  }
#endif

  // 폴링 간격 계산 (mTimeStep 단위로)
  int pollSteps = mPollInterval / mTimeStep;
  if (pollSteps < 1) pollSteps = 1;

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

    mGaitManager->step(mTimeStep);
    myStep();
  }
}

void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;

  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;

  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;

  if (fup > acc_step) {
    cout << "[낙상감지] 앞으로 넘어짐 - 복구중..." << endl;
    mMotionManager->playPage(10); // f_up
    mMotionManager->playPage(9);  // init position
    fup = 0;
  }
  else if (fdown > acc_step) {
    cout << "[낙상감지] 뒤로 넘어짐 - 복구중..." << endl;
    mMotionManager->playPage(11); // b_up
    mMotionManager->playPage(9);  // init position
    fdown = 0;
  }
}
