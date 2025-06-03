#include "walk_controller_mongodb.hpp"
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>

#include <curl/curl.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <fstream>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
  "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
  "AnkleL", "FootR", "FootL", "Neck", "Head"
};

// HTTP 응답을 저장하기 위한 콜백 함수
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
    ((string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

MongoDBWalkController::MongoDBWalkController() : Robot() {
    mTimeStep = getBasicTimeStep();
    initializeRobot();
}

MongoDBWalkController::~MongoDBWalkController() {
    delete mMotionManager;
    delete mGaitManager;
}

void MongoDBWalkController::initializeRobot() {
    // LED 초기화
    getLED("HeadLed")->set(0xFF0000);
    getLED("EyeLed")->set(0x00FF00);
    
    // 센서 초기화
    mAccelerometer = getAccelerometer("Accelerometer");
    mAccelerometer->enable(mTimeStep);
    
    mGyro = getGyro("Gyro");
    mGyro->enable(mTimeStep);
    
    mCamera = getCamera("Camera");
    mCamera->enable(mTimeStep);
    
    // 모터 및 위치 센서 초기화
    for (int i = 0; i < NMOTORS; i++) {
        mMotors[i] = getMotor(motorNames[i]);
        string sensorName = motorNames[i];
        sensorName.push_back('S');
        mPositionSensors[i] = getPositionSensor(sensorName);
        mPositionSensors[i]->enable(mTimeStep);
    }
    
    // 키보드 초기화
    mKeyboard = getKeyboard();
    mKeyboard->enable(mTimeStep);
    
    // ROBOTIS 매니저 초기화
    mMotionManager = new RobotisOp2MotionManager(this);
    mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
    
    cout << "Robot initialized successfully!" << endl;
}

void MongoDBWalkController::myStep() {
    int ret = step(mTimeStep);
    if (ret == -1)
        exit(EXIT_SUCCESS);
}

void MongoDBWalkController::wait(int ms) {
    double startTime = getTime();
    double s = (double)ms / 1000.0;
    while (s + startTime >= getTime())
        myStep();
}

// 실제 curl을 사용한 HTTP 요청
string MongoDBWalkController::httpRequest(const string& url, const string& data) {
    CURL *curl;
    CURLcode res;
    string readBuffer;
    
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        
        // POST 데이터가 있으면 설정
        if (!data.empty()) {
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, data.length());
            
            // JSON 헤더 설정
            struct curl_slist *headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        }
        
        // SSL 인증서 검증 비활성화 (개발용)
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
        
        // 요청 실행
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        
        if (res != CURLE_OK) {
            cerr << "HTTP request failed: " << curl_easy_strerror(res) << endl;
            return "";
        }
    }
    
    cout << "HTTP Request successful. Response length: " << readBuffer.length() << endl;
    return readBuffer;
}

bool MongoDBWalkController::loadMotionFromAPI(const string& api_url, const string& motion_id) {
    string url = api_url + "/motions/" + motion_id;
    cout << "Loading motion from: " << url << endl;
    
    string response = httpRequest(url);
    if (response.empty()) {
        cerr << "Failed to load motion from API" << endl;
        return false;
    }
    
    cout << "Motion data received: " << response.substr(0, 200) << "..." << endl;
    
    // JSON 파싱 (간단한 예시)
    mCurrentMotion = parseMotionFromJson(response);
    
    cout << "Motion loaded: " << mCurrentMotion.name << " with " 
         << mCurrentMotion.frames.size() << " frames" << endl;
    
    return true;
}

MotionSequence MongoDBWalkController::parseMotionFromJson(const string& json) {
    MotionSequence motion;
    
    // 간단한 JSON 파싱 (실제로는 더 정교한 파서 필요)
    motion.id = "sample_motion";
    motion.name = "Sample Walking Motion";
    
    // 샘플 모션 프레임 생성
    MotionFrame frame1;
    frame1.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    frame1.duration = 1000;
    frame1.name = "init_pose";
    
    motion.frames.push_back(frame1);
    
    return motion;
}

void MongoDBWalkController::runWithMongoDB() {
    cout << "=== MongoDB Walk Controller Started ===" << endl;
    cout << "Commands:" << endl;
    cout << "  SPACE: Start/Stop walking" << endl;
    cout << "  Arrow keys: Control direction" << endl;
    cout << "  P: Play loaded motion" << endl;
    cout << "  R: Record motion" << endl;
    cout << "  L: Log sensor data" << endl;
    cout << "  S: Save current motion to file" << endl;
    cout << "  F: Load motion from file" << endl;
    
    myStep(); // 첫 번째 스텝
    
    mMotionManager->playPage(9); // 초기 자세
    wait(200);
    
    bool isWalking = false;
    
    while (true) {
        checkIfFallen();
        logSensorData();
        
        mGaitManager->setXAmplitude(0.0);
        mGaitManager->setAAmplitude(0.0);
        
        // 키보드 입력 처리
        int key = 0;
        while ((key = mKeyboard->getKey()) >= 0) {
            switch (key) {
                case ' ': // Space bar - 걷기 시작/정지
                    if (isWalking) {
                        mGaitManager->stop();
                        isWalking = false;
                        cout << "Walking stopped" << endl;
                        wait(200);
                    } else {
                        mGaitManager->start();
                        isWalking = true;
                        cout << "Walking started" << endl;
                        wait(200);
                    }
                    break;
                    
                case Keyboard::UP: // 전진
                    mGaitManager->setXAmplitude(1.0);
                    break;
                    
                case Keyboard::DOWN: // 후진
                    mGaitManager->setXAmplitude(-1.0);
                    break;
                    
                case Keyboard::RIGHT: // 우회전
                    mGaitManager->setAAmplitude(-0.5);
                    break;
                    
                case Keyboard::LEFT: // 좌회전
                    mGaitManager->setAAmplitude(0.5);
                    break;
                    
                case 'P': // 모션 재생
                case 'p':
                    cout << "Playing loaded motion..." << endl;
                    playMotionSequence(mCurrentMotion);
                    break;
                    
                case 'R': // 모션 녹화
                case 'r':
                    cout << "Recording motion..." << endl;
                    recordMotionSequence("recorded_motion");
                    break;
                    
                case 'L': // 센서 데이터 로깅
                case 'l':
                    cout << "Logging sensor data..." << endl;
                    sendSensorDataToAPI("http://localhost:3000/api");
                    break;
                    
                case 'S': // 파일 저장
                case 's':
                    cout << "Saving motion to file..." << endl;
                    saveMotionToFile();
                    break;
                    
                case 'F': // 파일 로드
                case 'f':
                    cout << "Loading motion from file..." << endl;
                    cout << "Enter filename (e.g., motion_12345.json): ";
                    // 간단한 기본 파일명 사용
                    loadMotionFromFile("motion_sample.json");
                    break;
            }
        }
        
        mGaitManager->step(mTimeStep);
        myStep();
    }
}

void MongoDBWalkController::playMotionSequence(const MotionSequence& motion) {
    cout << "Playing motion: " << motion.name << endl;
    
    for (const auto& frame : motion.frames) {
        cout << "Playing frame: " << frame.name << endl;
        
        // 모터 위치 설정
        for (int i = 0; i < NMOTORS && i < static_cast<int>(frame.positions.size()); i++) {
            mMotors[i]->setPosition(frame.positions[i]);
        }
        
        wait(frame.duration);
    }
    
    cout << "Motion playback completed" << endl;
}

void MongoDBWalkController::recordMotionSequence(const string& motion_name) {
    cout << "Recording motion: " << motion_name << endl;
    cout << "Press SPACE to capture frames, ESC to finish" << endl;
    
    MotionSequence recorded_motion;
    recorded_motion.name = motion_name;
    recorded_motion.id = motion_name + "_" + to_string(time(nullptr));
    
    while (true) {
        myStep();
        
        int key = mKeyboard->getKey();
        if (key == ' ') {
            // 현재 모터 위치 캡처
            MotionFrame frame;
            frame.name = "frame_" + to_string(recorded_motion.frames.size());
            frame.duration = 1000;
            
            for (int i = 0; i < NMOTORS; i++) {
                frame.positions.push_back(mPositionSensors[i]->getValue());
            }
            
            recorded_motion.frames.push_back(frame);
            cout << "Frame captured: " << frame.name << endl;
            
        } else if (key == 27) { // ESC
            break;
        }
    }
    
    cout << "Recording completed with " << recorded_motion.frames.size() << " frames" << endl;
    
    // 파일로 저장
    mCurrentMotion = recorded_motion;
    saveMotionToFile();
}

bool MongoDBWalkController::saveMotionToAPI(const string& api_url, const MotionSequence& motion) {
    string url = api_url + "/motions";
    string json_data = motionToJson(motion);
    
    cout << "Saving motion to: " << url << endl;
    
    string response = httpRequest(url, json_data);
    if (response.empty()) {
        cerr << "Failed to save motion to API" << endl;
        return false;
    }
    
    cout << "Motion saved successfully!" << endl;
    return true;
}

string MongoDBWalkController::motionToJson(const MotionSequence& motion) {
    // 간단한 JSON 생성
    stringstream json;
    json << "{";
    json << "\"id\":\"" << motion.id << "\",";
    json << "\"name\":\"" << motion.name << "\",";
    json << "\"frames\":[";
    
    for (size_t i = 0; i < motion.frames.size(); i++) {
        const auto& frame = motion.frames[i];
        json << "{";
        json << "\"name\":\"" << frame.name << "\",";
        json << "\"duration\":" << frame.duration << ",";
        json << "\"positions\":[";
        
        for (size_t j = 0; j < frame.positions.size(); j++) {
            json << frame.positions[j];
            if (j < frame.positions.size() - 1) json << ",";
        }
        
        json << "]}";
        if (i < motion.frames.size() - 1) json << ",";
    }
    
    json << "]}";
    return json.str();
}

void MongoDBWalkController::saveMotionToFile() {
    string filename = "motion_" + mCurrentMotion.id + ".json";
    ofstream file(filename);
    if (file.is_open()) {
        file << motionToJson(mCurrentMotion);
        file.close();
        cout << "Motion saved to file: " << filename << endl;
    } else {
        cerr << "Failed to save motion to file" << endl;
    }
}

bool MongoDBWalkController::loadMotionFromFile(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return false;
    }
    
    stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    string json_content = buffer.str();
    mCurrentMotion = parseMotionFromJson(json_content);
    
    cout << "Motion loaded from file: " << filename << endl;
    cout << "Motion name: " << mCurrentMotion.name << endl;
    cout << "Frames: " << mCurrentMotion.frames.size() << endl;
    
    return true;
}

void MongoDBWalkController::logSensorData() {
    // 센서 데이터 수집
    const double* acc = mAccelerometer->getValues();
    const double* gyro = mGyro->getValues();
    
    mSensorData["timestamp"] = getTime();
    mSensorData["acc_x"] = acc[0];
    mSensorData["acc_y"] = acc[1];
    mSensorData["acc_z"] = acc[2];
    mSensorData["gyro_x"] = gyro[0];
    mSensorData["gyro_y"] = gyro[1];
    mSensorData["gyro_z"] = gyro[2];
    
    // 모터 위치들
    for (int i = 0; i < NMOTORS; i++) {
        string motor_key = "motor_" + to_string(i);
        mSensorData[motor_key] = mPositionSensors[i]->getValue();
    }
}

void MongoDBWalkController::sendSensorDataToAPI(const string& api_url) {
    string url = api_url + "/sensor-data";
    
    // 센서 데이터를 JSON으로 변환
    stringstream json;
    json << "{";
    bool first = true;
    for (const auto& pair : mSensorData) {
        if (!first) json << ",";
        json << "\"" << pair.first << "\":" << pair.second;
        first = false;
    }
    json << "}";
    
    string response = httpRequest(url, json.str());
    if (!response.empty()) {
        cout << "Sensor data sent successfully" << endl;
    }
    
    // 파일로도 저장
    static int log_counter = 0;
    string filename = "sensor_log_" + to_string(log_counter++) + ".json";
    ofstream file(filename);
    if (file.is_open()) {
        file << json.str();
        file.close();
        cout << "Sensor data also saved to: " << filename << endl;
    }
}

void MongoDBWalkController::checkIfFallen() {
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
        mMotionManager->playPage(10); // f_up
        mMotionManager->playPage(9);  // init position
        fup = 0;
        cout << "Robot fell forward - getting up" << endl;
    } else if (fdown > acc_step) {
        mMotionManager->playPage(11); // b_up
        mMotionManager->playPage(9);  // init position
        fdown = 0;
        cout << "Robot fell backward - getting up" << endl;
    }
}