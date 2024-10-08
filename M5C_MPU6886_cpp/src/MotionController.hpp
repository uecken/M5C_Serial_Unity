#define MotionController_H
#include <AccelRingBuffer.hpp>

//#define ESP32C3
#define M5STICKC_H

#if defined(ESP32C3) || defined(ESP32S3)
  #include <Arduino.h>
  #include "I2Cdev.h"
  #include "MPU6050_6Axis_MotionApps20.h"
  #include <Wire.h>
#elif defined(M5STICKC_H)
  //#include <Arduino.h>
  #include <M5StickC.h>
#endif

//#include "MotionController.h"
#define USE_NIMBLE
#ifdef USE_NIMBLE
#include <BleCombo.h>
BleCombo bleCombo;

//#include <BleKeyboard.h>
//#include <BleMouse.h>

#elif defined(!USE_NIMBLE)
#include <BleCombo.h>
#endif

const uint8_t P2_KEY_UP_ARROW = 'i';
const uint8_t P2_KEY_DOWN_ARROW = 'k';
const uint8_t P2_KEY_LEFT_ARROW = 'j';
const uint8_t P2_KEY_RIGHT_ARROW = 'l';

const String MODE_SF_P1 = "SF_P1";
const String MODE_SF_P2 = "SF_P2";




//Mode設定
#define NUM_OF_MODE 6
#define MODE_STREET_FIGHTER_DIST 0
#define MODE_STREET_FIGHTER_FOOT 1
#define MODE_PK3 2
#define MODE_MOTION_CONTROLLER 3  //MOTION INPUT
#define MODE_MOTION_MASSAGE 4  //MOTION MESSAGE
#define MODE_MOUSE 5
#define MODE_MOUSE_KEYBOARD 6
#define MODE_STREET_FIGHTER_RANGE 5 //not used


#define MODE_KEYBOARD 99
//#define MODE_XXXXX 9
#define MODE_TEST_BUTTONB 20


#define NO_EVENT 0
#define INPUT_EVENT 1
#define INPUT1_EVENT 2
#define INPUT2_EVENT 3
#define INPUT3_EVENT 4
#define REGISTRATION_ROLL_EVENT 5
#define REGISTRATION_PITCH_EVENT 6
#define REGISTRATION_ROLL_PITCH_EVENT 7
#define xxxxx_EVENT 8




struct SensorData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
};

// ローパスフィルタの係数
const float alpha = 0.9;

//extern BleComboMouse Mouse;
//extern BleComboKeyboard Keyboard;


//===========Player Key settings2=============

#include <vector>
struct pk2{
  uint16_t index;

  //===Trigger==
  uint8_t button_idx; //0~2
  short rpy[3]; //6byte
  float quatarnion[4]; //16byte
  float accX_triger;  
  float accY_triger;  
  float accZ_triger;   
  float accABS_trigger;
  float gyroX_triger;  
  float gyroY_triger;  
  float gyroZ_triger;
  float gyroABS_trigger;
  
  //==Output==
  char inputs_msg1; //ASCI+Key&Mouse or Gamepad
  char inputs_msgs[10]; //ASCI+Key&Mouse or Gamepad

  //==Option==
  uint8_t hid_input_interval; //default:20ms
  uint8_t msg_format; //0:deafult 1:Gamepad
};


struct basicInfo {
    String software_version;   // v0.1...
    String hardware_type;      // BurstMotion, M5StickC, M5Atom, Adapter_RP2040
    String hardware_version;   // v0.1...
    String initial_communication_mode;   // BLEHID_USBSerial, USBHID_BTSerial, BLEHID_BTSerial
    String motion_judge_way;   // Oyler, Quaternion
    String controller_hand;    // left-handed, right-handed
    bool serial_ON;            // 0, 1
    String Name_USB_HID;       // burstmotion
    String Name_BLE_HID;       // burstmotion
    String Name_BLE_Serial;    // burstmotion
};

// グローバル変数として basicInfo を定義
basicInfo info = {
    .software_version = SOFTWARE_VERSION,
    .hardware_type = HARDWARE_TYPE,
    .hardware_version = HARDWARE_VERSION,
    .initial_communication_mode = INITIAL_COMMUNICATION_MODE,
    .motion_judge_way = MOTION_JUDGE_WAY,
    .controller_hand = CONTROLLER_HAND,
    .serial_ON = SERIAL_ON,
    .Name_USB_HID = "burstmotion",
    .Name_BLE_HID = "burstmotion",
    .Name_BLE_Serial = "burstmotion"
};



struct pk3 {
  uint8_t index;
  uint8_t button_idx;
  uint8_t padding1[2];
  short rpy[3];
  uint8_t padding2[2];
  float quatarnion[4];
  float acc_triger[4];
  float gyro_triger[4];
  uint8_t inputs_msg[20];
  uint8_t hid_input_interval;
  uint8_t msg_format;
  uint8_t hid_input_acc_threshold;
  uint8_t padding3[1];
};



class MotionController{
    private:

    public:
        uint8_t event=0;
        bool events_bool[3]; 
        #if defined(ESP32C3) || defined(ESP32S3)
            MPU6050 mpu;
            //int16_t ax, ay, az;
            //int16_t gx, gy, gz;
            float ax, ay, az;
            float gx, gy, gz;
            float aOX, aOY, aOZ;
            float gOX, gOY, gOZ;
            uint8_t fifoBuffer[64]; // FIFO storage buffer
            Quaternion q;
        #elif defined(_M5STICKC_H_)
            float ax, ay, az;
            float gx, gy, gz;
            float aOX, aOY, aOZ;
            float gOX, gOY, gOZ;
            uint8_t fifoBuffer[64]; // FIFO storage buffer
            //Quaternion q;
        #endif

        #ifdef ILLUMITRACK_R
          #ifdef ESP32C3
            float lit;
            uint8_t RIGHT_LITTLE = 2;
            uint8_t button[1] = {RIGHT_LITTLE};
            //uint8_t button[3] = {0,26};s
            bool prev_button_state[1] = {false};
            uint8_t pin_LED = 99;
          #elif defined(ESP32S3)
            float lit;
            float rng;
            float mid;
            float idx;
            uint8_t RIGHT_LITTLE = 2;
            uint8_t RIGHT_RING = 3;
            uint8_t RIGHT_MIDDLE = 4;
            uint8_t RIGHT_INDEX = 5;
            uint8_t button[4] = {RIGHT_LITTLE,RIGHT_LITTLE,RIGHT_MIDDLE,RIGHT_INDEX};
            //uint8_t button[3] = {0,26};s
            bool prev_button_state[4] = {false,false,false,false};
            uint8_t pin_LED = LED_BUILTIN;
          #endif
        #elif defined(_M5STICKC_H_)


        //float mouse_speed = 10;
        uint8_t button[3] = {0,36,26};//LEFT_DISPLAY-> (26,36,0)
        boolean LEFT_DISPLAY = true; //0,26 button反
        //uint8_t button[3] = {};


        //uint8_t button[3] = {0,26};s
        bool prev_button_state[3] = {false,false,false};
        uint8_t pin_LED = 10;
        #endif 

        String game_modes[2] = {MODE_SF_P1,MODE_SF_P2};
        String current_game_mode = game_modes[0];
        uint8_t mode = 0;
        bool serial_ON=false;
        bool ble_enable=true;

        SensorData* sensorDataArray;
        int dataSize_;
        int currentDataIndex_;
        
        uint8_t mouse_scale2 = 2;
        float sampleFreq;
        float* q_offset = new float[4];
        boolean q_offset_enable = false;
        
        float initial_quat_horizontal[4] = {1,0,0,0}; // 寝かせ、ディスプレイ上の初期姿勢のクォータニオン
        float initial_quat_upright[4] = {0.444, 0.469, -0.532, -0.5}; // 右手持ち、ディスプレイ左の初期クォータニオン
        float* initial_quat = initial_quat_horizontal; //wxyz,M5Cにおける初期クォータニオン. MahonyFilterで設定されている

        //bool initial_quat_set = false;
        bool set_initial_quaternion = false;
        bool set_initial_quaternion_horizontal = false;
        bool set_initial_quaternion_upright = false;
        
        float rpy[3] = {0,0,0}; //Mahonyで初期値必要       
        float* q_array;

        std::vector<pk2> pk2_ref_vector;
        std::vector<pk3> pk3_ref_vector;

    // インスタンス作成時に構造体配列を初期化
        MotionController(int dataSize,float Fs) {
            sensorDataArray = new SensorData[dataSize];
            dataSize_ = dataSize;
            currentDataIndex_ = 0;
            sampleFreq = Fs;
        }

        // デストラクタでメモリ解放
        ~MotionController() {
            delete[] sensorDataArray;
        }

        // ローパスフィルタを適用する関数
        void applyLowPassFilter(float& filteredValue, float newValue) {
            filteredValue = alpha * filteredValue + (1 - alpha) * newValue;
        }

        // 取得したセンサデータを構造体に追加する関数
        void addSensorData(float accelX, float accelY, float accelZ, 
                                            float gyroX, float gyroY, float gyroZ, 
                                            float _roll, float _pitch, float _yaw) {
            //rpy[0] = roll;
            //rpy[1] = pitch;
            //rpy[2] = yaw;
            // ローパスフィルタを適用してデータを更新
            applyLowPassFilter(sensorDataArray[currentDataIndex_].accelX, accelX);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].accelY, accelY);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].accelZ, accelZ);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].gyroX, gyroX);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].gyroY, gyroY);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].gyroZ, gyroZ);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].roll, _roll);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].pitch, _pitch);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].yaw, _yaw);

            // インデックスを次に進める（ループする場合）
            currentDataIndex_ = (currentDataIndex_ + 1) % dataSize_;
        }



        void begin() {
        #ifdef ILLUMITRACK_R
          #ifdef ESP32C3
            pinMode(RIGHT_LITTLE, INPUT);
          #elif defined(ESP32S3)
            pinMode(RIGHT_LITTLE, INPUT);
            //pinMode(RIGHT_RING, INPUT);
            //pinMode(RIGHT_MIDDLE, INPUT);
            //pinMode(RIGHT_INDEX, INPUT);
          #endif
        #endif

        #if defined(ESP32C3) || defined(ESP32S3)
            Serial.begin(115200);
            Wire.begin();
            Serial.println("Initializing I2C devices...");
            mpu.initialize();
            Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

            int devStatus; // Declaration of the devStatus variable
            devStatus = mpu.dmpInitialize();      
            if (devStatus == 0) {
                mpu.setDMPEnabled(true);
                uint16_t  packetSize = mpu.dmpGetFIFOPacketSize();
                bool dmpReady = true;
            } else {
                // Handle DMP initialization failure
                Serial.println("DMP initialization failed");
            }

            if(ble_enable){
                bleCombo.setName("MC_XIAOC3");
                bleCombo.begin();     
                while(!bleCombo.isConnected()){
                    vTaskDelay(100);
                    Serial.println("BLE Connecting...");
                }

                    Serial.println("BLE Connected!");
            }else{
                Serial.println("BLE disabled");
            }
        #else 
            M5.begin();
            M5.Axp.ScreenBreath(8);//省電力化

            delay(500);
            M5.Imu.Init(sampleFreq);  //Init IMU.  
            M5.Lcd.setRotation(3);  //Rotate the screen. 
            M5.Lcd.fillScreen(BLACK);
            M5.Lcd.setTextSize(1);
            M5.Lcd.setCursor(5, 5); //set the cursor location. 
            M5.Lcd.println("Motion Controller");
            //M5.Lcd.setCursor(30, 30);
            //M5.Lcd.println("  X       Y       Z");
            M5.Lcd.setCursor(5, 15);
            M5.Lcd.println("Mode:");
            changeMCMode(true);

            M5.Lcd.setCursor(5, 25);
            M5.Lcd.println("GameMode:");
            changeGameMode(true);

            M5.Lcd.setCursor(5, 35); 
            M5.Lcd.println("BLE:");

            M5.Lcd.setCursor(15, 50); 
            M5.Lcd.println("  Pitch   Roll    Yaw");

            M5.Lcd.setCursor(5, 72);
            M5.Lcd.println("Serial:");
            changeSerialONOFF(true);

            M5.Lcd.setCursor(100, 72);
            M5.Lcd.print(sampleFreq);

            pinMode(pin_LED,   OUTPUT); //LED
            digitalWrite(pin_LED, HIGH); //OFF
            pinMode(button[0],   INPUT_PULLUP); //PU 起動モード設定のためか、常にプルアップされており、ソフト制御できない。スイッチでGNDに落として判定する必要あり。
            pinMode(button[1],   INPUT); //3.3V入力をスイッチとして利用可能
            pinMode(button[2],   INPUT_PULLUP); //半PD 電圧が1V程にプルダウン？されている。Pin36を ONにするとノイズが発生し、GNDとの判定で誤判定となる。3.3Vとのショート判定する必要がある。
            //pinMode(button[1],   INPUT_PULLUP);//26

            if(LEFT_DISPLAY){
                uint8_t button_tmp = button[0];
                button[0] = button[2];
                button[2] = button_tmp;
            }

            if(ble_enable){
                bleCombo.setName("MotionController");
                bleCombo.begin();
                //bleCombo.begin(); 
                //bleCombo.setDelay(7);           
                while(!bleCombo.isConnected()){
                    vTaskDelay(100);
                    M5.Lcd.setCursor(5, 35);
                    M5.Lcd.print("BLE Connecting...");
                }
                M5.Lcd.setCursor(5, 35);
                M5.Lcd.print("BLE Connected!      ");
            }else{
                M5.Lcd.setCursor(5, 35);
                M5.Lcd.print("BLE disabled      ");
            }

            #endif
        }

        void set_qoffset(float* q_array){
            for (int i = 0; i < 4; ++i) {
                q_offset[i] = q_array[i];
            }
            q_offset_enable = true;
        }

        // 初期姿勢のクォータニオンを設定する関数
        void setInitialQuat(float* quat) {
            for (int i = 0; i < 4; ++i) {
                initial_quat[i] = quat[i];
            }
            //set_initial_quaternion = true;
            M5.IMU.setQuaternion(quat);
        }


        void quatMultiply(float* Q1, float* Q2) {
            // Conjugate the offset quaternion to reverse its rotation
            float Q2_conjugate[4] = { Q2[0], -Q2[1], -Q2[2], -Q2[3] };

            float w = Q1[0] * Q2_conjugate[0] - Q1[1] * Q2_conjugate[1] - Q1[2] * Q2_conjugate[2] - Q1[3] * Q2_conjugate[3];
            float x = Q1[0] * Q2_conjugate[1] + Q1[1] * Q2_conjugate[0] + Q1[2] * Q2_conjugate[3] - Q1[3] * Q2_conjugate[2];
            float y = Q1[0] * Q2_conjugate[2] - Q1[1] * Q2_conjugate[3] + Q1[2] * Q2_conjugate[0] + Q1[3] * Q2_conjugate[1];
            float z = Q1[0] * Q2_conjugate[3] + Q1[1] * Q2_conjugate[2] - Q1[2] * Q2_conjugate[1] + Q1[3] * Q2_conjugate[0];
            Q1[0] = w; Q1[1] = x; Q1[2] = y; Q1[3] = z;
        }  

        /*
                // クォータニオンの逆を計算する関数
        void inverseQuat(float* quat, float* result) {
            float norm = quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3];
            result[0] = quat[0] / norm;
            result[1] = -quat[1] / norm;
            result[2] = -quat[2] / norm;
            result[3] = -quat[3] / norm;
        }


        // 初期姿勢に戻す関数
        void resetToInitialQuat(float* current_quat) {
            if (!initial_quat_set) return;
            float inverse_initial_quat[4];
            inverseQuat(initial_quat, inverse_initial_quat);
            float result[4];
            quatMultiply(current_quat, inverse_initial_quat, result);
            for (int i = 0; i < 4; ++i) {
                current_quat[i] = result[i];
            }
        }
        */


        void reConnect() {
            bleCombo.end();
            //bleCombo.end();
            bleCombo.begin();
            //bleCombo.begin();

            #ifdef _M5STICKC_H_
            M5.Lcd.print("BLE Connecting....");
            while(!bleCombo.isConnected()){
                M5.Lcd.print("BLE Connecting....");
                vTaskDelay(100);
            }
            Serial.println("BLE Connected!");
            #endif
            while(!bleCombo.isConnected()){
                Serial.println("BLE Connecting....");
                vTaskDelay(100);
            }
            Serial.println("BLE Connected!");

        }


        void inputKey(char key){
            if(bleCombo.isConnected()){
                //bleCombo.println(key);   
                bleCombo.write(key);
            }else{
                Serial.println("Keyboard is not connected.");
            }
        }

        void inputKeys(String keys){
            if(bleCombo.isConnected()){
                bleCombo.println(keys);
                bleCombo.releaseAll();
            }else{
                Serial.println("Keyboard is not connected.");
            }
        }


        void pressKey(char key){
            if(bleCombo.isConnected()){
                //bleCombo.press(key);
                bleCombo.press(uint8_t(key));
            }else{
                Serial.println("Keyboard is not connected.");
            }
        }

        void releaseKey(char key){
            if(bleCombo.isConnected()){
                //bleCombo.release(key);
                bleCombo.release(uint8_t(key));
            }
        }

        void keyboardReleaseAll(){
            if(bleCombo.isConnected()){
                bleCombo.releaseAll();
            }
        }


        void moveMouse(float x, float y){
            //x = x * mouse_speed;
            //y = y * mouse_speed;
            /*
            if(x>127) x=127;
            else if(x <-127) x=-127;
            if(y>127) y=127;
            else if(y <-127) y=-127;
            */
            
            bleCombo.move(x,y);
            return;
        }

/*
        void moveMouseSmoothly(float x, float y, float timeDelta){

            float mouse_scale = 10; // 感度調整
            int xMove = round(mouse_scale * yawDelta / timeDelta);
            int yMove = round(mouse_scale * pitchDelta / timeDelta);
            mc.moveMouse(xMove, yMove);

            bleCombo.move(x,y);
            return;
        }
*/

        /*
        void setMouseSpeed(float speed){
            mouse_speed = speed;
        }
        */

        bool switchRead(uint8_t button){
            if(button==0){ //Normally pull-up
                if(digitalRead(button) == HIGH){
                    return false;
                    //Serial.println("button:"+String(button));
                }else{
                    return true;
                }
            }
            else if(button==36){
                if(digitalRead(button) == LOW){
                    return false;
                    //Serial.println("button:"+String(button));
                }else{
                    return true;
                }
            }
            else if(button==26){
                if(digitalRead(button) == HIGH){
                    return false;
                    //Serial.println("button:"+String(button));
                }else{
                    return true;
                }
            }

            return true;
        }


        //For Street Fighter
        void execSF_HIDInputs(char* inputs,float acc_threshold, AccelRingBuffer* acc_buffer, bool pre_acc_flag){
            //文字列終端までfor文を実行する
            int i = 0;
            bool acc_flag = false;


            digitalWriteLED(true);
            /* 5/1 remove for Stick Command
            if(strlen(inputs)<=4){
                delay(100);
            }
            */

            if(!bleCombo.isConnected()){
                Serial.println("BLE is not connected.");
            }

            while(inputs[i]!='\0'){
                if((acc_buffer->getMaxAbsAccel(5)) >= acc_threshold || pre_acc_flag) acc_flag = true;
                
                //過去入力ボタンのリリース
                //(波動拳等のコマンド入力で初期入力の↓がリリースされている必要がある)
                if(i>=2){
                    //bleCombo.release(inputs[i-2]);
                    bleCombo.release(uint8_t(inputs[i-2]));
                    vTaskDelay(20);
       
                }

                //最終入力値以外の通常入力
                if(inputs[i+1]!='\0'){
                    //bleCombo.press(inputs[i]);
                    bleCombo.press(uint8_t(inputs[i]));
                    vTaskDelay(30);
                }
                //最終入力値は一定以上の閾値が観測された場合に実行する
                else if(inputs[i+1]=='\0'){
                    if(acc_flag){
                        digitalWriteLED(false);
                        //bleCombo.press(inputs[i]);
                        bleCombo.press(uint8_t(inputs[i]));
                        acc_flag = false;
                    }
                    vTaskDelay(30);
                    digitalWriteLED(true); 
                }
                i++;
            }


            if(inputs[i]=='\0'){
                vTaskDelay(50);
                bleCombo.releaseAll();
                digitalWriteLED(false);
            }

        }


        void changeMCMode(boolean init){
            /*
            if(mc.mode==MODE_MOUSE){
                mc.mode = MODE_MOTION_CONTROLLER
;
                if(DEBUG_HID) Serial.println("MODE_MOTION_CONTROLLER");

            }else if(mc.mode==MODE_MOTION_CONTROLLER){
                mc.mode = MODE_STREET_FIGHTER;
                if(DEBUG_HID) Serial.println("MODE_STREET_FIGHTER");

            }else if(mc.mode==MODE_STREET_FIGHTER){
                mc.mode = MODE_MOUSE;
                if(DEBUG_HID) Serial.println("MODE_MOUSE");
            }
            */
            if(!init) mode = (mode + 1)% NUM_OF_MODE;
            String mode_str;
            switch(mode){
                case MODE_STREET_FIGHTER_DIST: mode_str="STREET FIGHTER      "; break;
                case MODE_STREET_FIGHTER_FOOT: mode_str="STREET FIGHTER FOOT     "; break;
                case MODE_PK3: mode_str                ="PK3                 "; break;                
                case MODE_MOTION_CONTROLLER: mode_str  ="MOTION CONTROLLER   "; break;
                case MODE_MOTION_MASSAGE: mode_str     ="MOTION_MASSAGE      "; break;  
                case MODE_MOUSE: mode_str              ="MOUSE               "; break;
                //case MODE_MOUSE_KEYBOARD: mode_str     ="MOUKEY              "; break;
                //case MODE_STREET_FIGHTER_RANGE:mode_str="STREET FIGHTER OLD  "; break;

                default: mode_str="UNKNOWN      "; break;
            }
            #ifdef _M5STICKC_H_
            M5.Lcd.setCursor(5, 15);
            M5.Lcd.println("Mode:"+mode_str);
            #elif defined(ESP32C3)
            Serial.println("Mode:"+mode_str);
            #endif
    
        }

        void changeGameMode(boolean init)
        {
            if(!init){
                if(mode==MODE_STREET_FIGHTER_DIST || mode==MODE_STREET_FIGHTER_RANGE ){
                    if(current_game_mode == game_modes[0]){
                        current_game_mode = game_modes[1];
                    }else{
                        current_game_mode = game_modes[0];
                    }
                }
            }
            #ifdef _M5STICKC_H_
            M5.Lcd.setCursor(5, 25); 
            M5.Lcd.print("GameMode:"+current_game_mode);
            #elif defined(ESP32C3)
            Serial.println("GameMode:"+current_game_mode);
            #endif

        }

        void changeSerialONOFF(bool init){
            //String serial_str;
            if(!init){
                if(serial_ON){
                    //M5.Lcd.println("Serial:OFF");
                    serial_ON = false;
                    //serial_str = "OFF";
                }
                else if(!serial_ON){
                    //M5.Lcd.println("Serial:ON ");
                    serial_ON = true;
                    //serial_str = "ON";
                }
            }
            #ifdef _M5STICKC_H_
            M5.Lcd.setCursor(5, 70);
            M5.Lcd.print("Serial:"+String(serial_ON));
            #elif defined(ESP32C3)
            Serial.println("Serial:"+String(serial_ON));
            #endif
        }

        void serialON(){
            serial_ON = true;
            #ifdef _M5STICKC_H_
            M5.Lcd.setCursor(5, 70);
            M5.Lcd.print("Serial:"+String(serial_ON));
            #elif defined(ESP32C3)
            Serial.println("Serial:"+String(serial_ON));
            #endif

        }
        void serialOFF(){
            serial_ON = false;
            #ifdef _M5STICKC_H_
            M5.Lcd.setCursor(5, 70);
            M5.Lcd.print("Serial:"+String(0));
            #elif defined(ESP32C3)
            Serial.println("Serial:"+String(serial_ON));
            #endif
        }


        void SF_hadoken(){
                bleCombo.press(KEY_DOWN_ARROW);
                vTaskDelay(50);
                bleCombo.press(KEY_RIGHT_ARROW);
                vTaskDelay(50);

                bleCombo.release(KEY_DOWN_ARROW);
                //bleCombo.press(KEY_RIGHT_ARROW);
                bleCombo.press(uint8_t(KEY_RIGHT_ARROW));
                vTaskDelay(50);

                //bleCombo.press('a');
                bleCombo.press(uint8_t('a'));
                //TaskDelay(50);
        }

        void digitalWriteLED(bool state){
            if(state==true){digitalWrite(pin_LED, LOW);} //ON
            else{digitalWrite(pin_LED, HIGH);} //OFF
        }


        //========= PK2 =============


        pk2 pk2_array2struct(String inputs[],uint8_t start_idx) {
            /*for (int i = 0; i < sizeof(inputs) / sizeof(inputs[0]); ++i) {
                Serial.println("," + inputs[i]);
            }*/
            Serial.println("5");
            Serial.println(inputs[0]);
            Serial.println(inputs[1]);
            Serial.println(inputs[2]);
            pk2 newPK2;
            
            newPK2.index = inputs[0+start_idx].toInt();
            newPK2.button_idx = inputs[1+start_idx].toInt();
            for (int i = 0; i < 3; i++) {
                newPK2.rpy[i] = inputs[2 + i +start_idx].toInt();
            }
            for (int i = 0; i < 4; i++) {
                newPK2.quatarnion[i] = inputs[5 + i +start_idx].toFloat();
            }
            newPK2.accX_triger = inputs[9+start_idx].toFloat();
            newPK2.accY_triger = inputs[10+start_idx].toFloat();
            newPK2.accZ_triger = inputs[11+start_idx].toFloat();
            newPK2.accABS_trigger = inputs[12+start_idx].toFloat();
            newPK2.gyroX_triger = inputs[13+start_idx].toFloat();
            newPK2.gyroY_triger = inputs[14+start_idx].toFloat();
            newPK2.gyroZ_triger = inputs[15+start_idx].toFloat();
            newPK2.gyroABS_trigger = inputs[16+start_idx].toFloat();
  
            /*
            String example_msg = inputs[17 + start_idx];
            for (int i = 0; i < 20; ++i) {
                if (i < example_msg.length()) {
                    newPK2.inputs_msg[i] = static_cast<uint8_t>(example_msg[i]);
                } else {
                    newPK2.inputs_msg[i] = 0; // Default value if input is out of range
                }
            }
            */
            newPK2.inputs_msg1 = inputs[17+start_idx].c_str()[0];
            newPK2.hid_input_interval = inputs[18+start_idx].toInt(); // Set default interval to 20ms
            newPK2.msg_format = inputs[19+start_idx].toInt();


            Serial.println("6:");
            Serial.println(newPK2.index);
            Serial.println(newPK2.button_idx);
            serialprint_pk2_csv(newPK2);
            Serial.println("7:");

            return newPK2;
        }


        void addPK2ToVector(pk2 newPK2, std::vector<pk2> pk2_vector) {
            pk2_vector.push_back(newPK2);
        }
        
        void readPK2vector(String fileName, std::vector<pk2>* pk2_vector) {
            // Open the file with the specified file name
            File file = LittleFS.open("/" + fileName, "r");
            if (!file) {
                Serial.println("Failed to open file for reading");
                return;
            }

            // Read the number of elements in the vector
            uint8_t num_elements;
            file.read((uint8_t*)&num_elements, sizeof(num_elements));

            // Clear the existing vector
            pk2_vector->clear();

            // Read each pk2 structure from the file and add to the vector
            for (uint8_t i = 0; i < num_elements; ++i) {
                pk2 pk2_element;
                file.read((uint8_t*)&pk2_element, sizeof(pk2_element));
                pk2_vector->push_back(pk2_element);
            }

            // Close the file
            file.close();
            Serial.println("pk2_vector read from LittleFS successfully");
        }

        void flushPK2vector(std::vector<pk2>* pk2_vector, String fileName) {
            // Create or open the file with the specified file name
            File file = LittleFS.open("/" + fileName, "w");
            if (!file) {
                Serial.println("Failed to open file for writing");
                return;
            }

            // Write the number of elements in the vector
            uint8_t num_elements = pk2_vector->size();
            file.write((uint8_t*)&num_elements, sizeof(num_elements));

            // Write each pk2 structure to the file
            for (const auto& pk2_element : *pk2_vector) {
                file.write((uint8_t*)&pk2_element, sizeof(pk2_element));
            }

            // Close the file
            file.close();
            Serial.println("pk2_vector written to LittleFS successfully");
        }

        void serialprint_pk2vector(std::vector<pk2>* pk2_vector) {
            for (const auto& pk2_element : *pk2_vector) {
                serialprint_pk2_csv(pk2_element);
            }
        }

        // The inputs_msg field in the pk2 struct is an array of uint8_t, which represents ASCII characters.
        // Therefore, the correct printf format specifier for this field should be %s, as it is used to print strings.
        // However, since inputs_msg is an array of uint8_t, we need to ensure it is null-terminated before printing.
        // Here is the corrected code:

        /*
        void serialprint_pk2_csv(pk2 pk2_element) {
            // Ensure inputs_msg is null-terminated
            char inputs_msg_str[21];
            memcpy(inputs_msg_str, pk2_element.inputs_msg, 20);
            inputs_msg_str[20] = '\0';

            Serial.println("serialprint_pk2_csv:");
            Serial.printf(
                "pk2,%u,%u,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%s,%u,%u\n",
                pk2_element.index,
                pk2_element.button_idx,
                pk2_element.rpy[0], pk2_element.rpy[1], pk2_element.rpy[2],
                pk2_element.quatarnion[0], pk2_element.quatarnion[1], pk2_element.quatarnion[2], pk2_element.quatarnion[3],
                pk2_element.accX_triger, pk2_element.accY_triger, pk2_element.accZ_triger, pk2_element.accABS_trigger,
                pk2_element.gyroX_triger, pk2_element.gyroY_triger, pk2_element.gyroZ_triger, pk2_element.gyroABS_trigger,
                inputs_msg_str,
                pk2_element.hid_input_interval,
                pk2_element.msg_format
            );
        }*/
        
/*
        void serialprint_pk2_csv(pk2 pk2_element) {
            Serial.println("serialprint_pk2_csv_hex:");
            Serial.printf(
                "pk2,%u,%u,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,",
                pk2_element.index,
                pk2_element.button_idx,
                pk2_element.rpy[0], pk2_element.rpy[1], pk2_element.rpy[2],
                pk2_element.quatarnion[0], pk2_element.quatarnion[1], pk2_element.quatarnion[2], pk2_element.quatarnion[3],
                pk2_element.accX_triger, pk2_element.accY_triger, pk2_element.accZ_triger, pk2_element.accABS_trigger,
                pk2_element.gyroX_triger, pk2_element.gyroY_triger, pk2_element.gyroZ_triger, pk2_element.gyroABS_trigger
            );

            // Print inputs_msg as hex
            
            for (int i = 0; i < 20; ++i) {
                Serial.printf("%02X", pk2_element.inputs_msg[i]);
                if (i < 19) {
                    Serial.print(" ");
                }
            }

            Serial.printf(",%u,%u\r\n",
                pk2_element.hid_input_interval,
                pk2_element.msg_format
            );
        }
*/



        void serialprint_pk2_csv(pk2 pk2_element) {
            Serial.println("serialprint_pk2_csv:");
            Serial.printf(
                "pk2,%u,%u,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%c,%u,%u\r\n",
                pk2_element.index,
                pk2_element.button_idx,
                pk2_element.rpy[0], pk2_element.rpy[1], pk2_element.rpy[2],
                pk2_element.quatarnion[0], pk2_element.quatarnion[1], pk2_element.quatarnion[2], pk2_element.quatarnion[3],
                pk2_element.accX_triger, pk2_element.accY_triger, pk2_element.accZ_triger, pk2_element.accABS_trigger,
                pk2_element.gyroX_triger, pk2_element.gyroY_triger, pk2_element.gyroZ_triger, pk2_element.gyroABS_trigger, //17
                pk2_element.inputs_msg1,
                pk2_element.hid_input_interval,
                pk2_element.msg_format
            );
        }


        //========= LittleFS =============
        void deleteLittleFSFile(String fileName) {
            // Check if the file exists
            if (LittleFS.exists("/" + fileName)) {
                // Delete the file
                if (LittleFS.remove("/" + fileName)) {
                    Serial.println("File deleted successfully");
                } else {
                    Serial.println("Failed to delete the file");
                }
            } else {
                Serial.println("File does not exist");
            }
        }


        //========= basicInfo =============
        //Webから受信したbasicInfo配列をbasic_info.binへ書き込む
        //Format:Software Version,Hardware Type,Hardware Version,Initial Communication Mode,Motion Judge Way,Controller Hand,Serial ON,Name USB HID,Name BLE HID,Name BLE Serial
        void flushBasicInfo(String infos[]) {
            // Create or open the file for writing
            File file = LittleFS.open("/basic_info.bin", "w");
            if (!file) {
                Serial.println("Failed to open basic_info.bin for writing");
                return;
            }
            Serial.println("Succeed writing basic_info.bin");

            // Write the basicInfo structure to the file
            //basicInfo info;
            info.software_version = infos[1].toInt();
            info.hardware_type = infos[2].toInt();
            info.hardware_version = infos[3].toInt();
            info.initial_communication_mode = infos[4].toInt();
            info.motion_judge_way = infos[5].toInt();
            info.controller_hand = infos[6].toInt();
            info.serial_ON = infos[7].toInt();
            info.Name_USB_HID = infos[7];
            info.Name_BLE_HID = infos[8];
            info.Name_BLE_Serial = infos[9];

            file.write((uint8_t*)&info, sizeof(info));

            // Close the file
            file.close();
            Serial.println("basicInfo written to LittleFS successfully");
        }

        //basic_info.binからBasinInfoを読み込に、infoに格納する
        void loadBasicInfo(){
            // Open the file for reading
            File file = LittleFS.open("/basic_info.bin", "r");
            if (!file) {
                Serial.println("Failed to open basic_info.bin for reading");
                return;
            }

            // Read the basicInfo structure from the file
            //basicInfo info;
            file.read((uint8_t*)&info, sizeof(info));

            // Close the file
            file.close();
        }

        void serialPrintBasicInformation() {
            // Print the basicInfo structure
            Serial.println("Basic Information:");
            Serial.printf("Software Version: %d\n", info.software_version);
            Serial.printf("Hardware Type: %d\n", info.hardware_type);
            Serial.printf("Hardware Version: %d\n", info.hardware_version);
            Serial.printf("Initial Communication Mode: %d\n", info.initial_communication_mode);
            Serial.printf("Motion Judge Way: %d\n", info.motion_judge_way);
            Serial.printf("Controller Hand: %d\n", info.controller_hand);
            Serial.printf("Serial ON: %d\n", info.serial_ON);
            Serial.printf("Name USB HID: %s\n", info.Name_USB_HID);
            Serial.printf("Name BLE HID: %s\n", info.Name_BLE_HID);
            Serial.printf("Name BLE Serial: %s\n", info.Name_BLE_Serial);
        }

        void serialOutBasicInfo() {
            //String csvHeader = "Software Version,Hardware Type,Hardware Version,Initial Communication Mode,Motion Judge Way,Controller Hand,Serial ON,Name USB HID,Name BLE HID,Name BLE Serial";
            String csvHeader = "BasicInfo";
            String csvData = info.software_version + "," + 
                            info.hardware_type + "," + 
                            info.hardware_version + "," + 
                            info.initial_communication_mode + "," + 
                            info.motion_judge_way + "," + 
                            info.controller_hand + "," + 
                            (info.serial_ON ? "1" : "0") + "," + 
                            info.Name_USB_HID + "," + 
                            info.Name_BLE_HID + "," + 
                            info.Name_BLE_Serial;

            // シリアルモニタに出力
            Serial.println(csvHeader+","+csvData);
        }        

        // LittleFSにpk3ベクトルを保存する関数
        bool savePk3VectorToLittleFS(const char* path, const std::vector<pk3>& data) {
            String safePath = path;
            if (!safePath.startsWith("/")) {
                safePath = "/" + safePath;
            }
            
            File file = LittleFS.open(safePath.c_str(), FILE_WRITE);
            if (!file) {
                Serial.println("Failed to open file for writing");
                return false;
            }
            size_t dataSize = data.size();
            size_t totalSize = dataSize * sizeof(pk3);
            file.write((uint8_t*)&dataSize, sizeof(size_t));
            file.write((uint8_t*)&totalSize, sizeof(size_t));
            //file.write((uint8_t*)data.data(), totalSize);
            size_t dataWritten = file.write((uint8_t*)data.data(), totalSize);
            if (dataWritten != totalSize) {
                Serial.printf("Failed to write data. Expected %u bytes, wrote %u bytes\n", totalSize, dataWritten);
                file.close();
                return false;
            }            
            file.close();
            Serial.println("Data written to LittleFS");
            return true;
        }


    bool loadpk3Vector(const char *path, std::vector<pk3>& data) {
        String safePath = path;
        if (!safePath.startsWith("/")) {
            safePath = "/" + safePath;
        }

        File file = LittleFS.open(safePath.c_str(), FILE_READ);
        if (!file) {
            Serial.println("Failed to open file for reading");
            return false;
        }

        size_t dataSize;
        size_t totalSize;
        file.read((uint8_t*)&dataSize, sizeof(size_t));
        file.read((uint8_t*)&totalSize, sizeof(size_t));

        data.clear();
        data.resize(dataSize);

        if (file.read((uint8_t*)data.data(), totalSize) != totalSize) {
            Serial.println("Failed to read data from file");
            file.close();
            return false;
        }

        file.close();
        Serial.print("Data size: "); Serial.println(dataSize);
        Serial.print("Total size: "); Serial.println(totalSize);
        Serial.println("Data read from LittleFS");
        return true;
    }

    void printpk3Vector(const std::vector<pk3>& data) {
        Serial.println("PK3 Vector contents:");
        for (size_t i = 0; i < data.size(); ++i) {
            const pk3& item = data[i];
            Serial.printf("PK3 %d:\n", i);
            Serial.printf("  Index: %d\n", item.index);
            Serial.printf("  Button Index: %d\n", item.button_idx);
            Serial.printf("  RPY: %d, %d, %d\n", item.rpy[0], item.rpy[1], item.rpy[2]);
            Serial.printf("  Quaternion: %.2f, %.2f, %.2f, %.2f\n", 
                item.quatarnion[0], item.quatarnion[1], item.quatarnion[2], item.quatarnion[3]);
            Serial.printf("  Acc Trigger: %.2f, %.2f, %.2f, %.2f\n", 
                item.acc_triger[0], item.acc_triger[1], item.acc_triger[2], item.acc_triger[3]);
            Serial.printf("  Gyro Trigger: %.2f, %.2f, %.2f, %.2f\n", 
                item.gyro_triger[0], item.gyro_triger[1], item.gyro_triger[2], item.gyro_triger[3]);
            Serial.print("  Inputs Message: ");
            for (int j = 0; j < 20; ++j) {
                if (item.inputs_msg[j] != 0) {
                    Serial.print((char)item.inputs_msg[j]);
                } else {
                    break;
                }
            }
            Serial.println();
            Serial.printf("  HID Input Interval: %d\n", item.hid_input_interval);
            Serial.printf("  Message Format: %d\n", item.msg_format);
            Serial.println();
        }
    }

    // ディレクトリ確認関数
    void listRootDirectory() {
        Serial.println("Listing root directory contents:");

        File root = LittleFS.open("/");
        if (!root) {
            Serial.println("Failed to open root directory");
            return;
        }
        if (!root.isDirectory()) {
            Serial.println("Not a directory");
            return;
        }

        File file = root.openNextFile();
        while (file) {
            if (file.isDirectory()) {
                Serial.printf("  DIR : %s\n", file.name());
            } else {
                Serial.printf("  FILE: %s  SIZE: %d\n", file.name(), file.size());
            }
            file = root.openNextFile();
        }

        root.close();
        Serial.println("Root directory listing complete");
    }


    // 指定パスに文字列を書き込む関数
    bool writeStringToFile(const char* path, const String& content) {
        String safePath = ensureLeadingSlash(path);
        File file = LittleFS.open(safePath, FILE_WRITE);
        if (!file) {
            Serial.printf("Failed to open file for writing: %s\n", safePath.c_str());
            return false;
        }

        size_t bytesWritten = file.print(content);
        file.close();

        if (bytesWritten == content.length()) {
            Serial.printf("File written: %s\n", safePath.c_str());
            return true;
        } else {
            Serial.printf("Write failed: %s\n", safePath.c_str());
            return false;
        }
    }

    // 指定パスから文字列を読み取る関数
    String readStringFromFile(const char* path) {
        String safePath = ensureLeadingSlash(path);
        File file = LittleFS.open(safePath, FILE_READ);
        if (!file) {
            Serial.printf("Failed to open file for reading: %s\n", safePath.c_str());
            return String();
        }

        String content = file.readString();
        file.close();

        Serial.printf("File read from: %s\n", safePath.c_str());
        Serial.printf("Content: %s\n", content.c_str());  // 読み取った内容を表示
        return content;
    }

    String ensureLeadingSlash(const char* path) {
        String safePath = path;
        if (!safePath.startsWith("/")) {
            safePath = "/" + safePath;
        }
        return safePath;
    }

    pk3 getClosestPK3(std::vector<pk3> pk3_refs, String type) {
        pk3 pk3_selected;
        float distance;
        float min_distance = 999999.0f;

        if (type == "rp") {
            for (uint8_t i = 0; i < pk3_refs.size(); i++) {
            short roll_ref = pk3_refs[i].rpy[0];
            short pitch_ref = pk3_refs[i].rpy[1];

            float roll_diff = rpy[0] - roll_ref;
            if (rpy[0] > 90 && roll_ref < -90) {
                roll_diff = (180 - rpy[0]) + (180 + roll_ref);
            } else if (rpy[0] < -90 && roll_ref > 90) {
                roll_diff = -(-180 - rpy[0]) + (180 - roll_ref);
            }

            float pitch_diff = rpy[1] - pitch_ref;

            distance = sqrt(roll_diff * roll_diff + pitch_diff * pitch_diff);

                if (distance < min_distance) {
                    min_distance = distance;
                    pk3_selected = pk3_refs[i];
                }
            }
        } else if (type == "quaternion") {
            for (uint8_t i = 0; i < pk3_refs.size(); i++) {
                float qx_diff = (*q_array) - pk3_refs[i].quatarnion[0];
                float qy_diff = (*(q_array + 1)) - pk3_refs[i].quatarnion[1];
                float qz_diff = (*(q_array + 2)) - pk3_refs[i].quatarnion[2];
                float qw_diff = (*(q_array + 3)) - pk3_refs[i].quatarnion[3];

                distance = sqrt(qx_diff * qx_diff + qy_diff * qy_diff + qz_diff * qz_diff + qw_diff * qw_diff);

                if (distance < min_distance) {
                    min_distance = distance;
                    pk3_selected = pk3_refs[i];
                }
            }
        }

        return pk3_selected;
    }


    const char* calibrationFile = "/calibration.dat";

    void saveCalibrationData() {
    File file = LittleFS.open(calibrationFile, "w");
    if(!file){
        Serial.println("Failed to open calibration file for writing");
        return;
    }
    
    file.write((uint8_t*)&aOX, sizeof(aOX));
    file.write((uint8_t*)&aOY, sizeof(aOY));
    file.write((uint8_t*)&aOZ, sizeof(aOZ));
    file.write((uint8_t*)&gOX, sizeof(gOX));
    file.write((uint8_t*)&gOY, sizeof(gOY));
    file.write((uint8_t*)&gOZ, sizeof(gOZ));
    
    file.close();
    Serial.println("Calibration data saved to LittleFS");
    }

    bool loadCalibrationData() {
    File file = LittleFS.open(calibrationFile, "r");
    if(!file){
        Serial.println("No calibration file found");
        return false;
    }
    
    if(file.read((uint8_t*)&aOX, sizeof(aOX)) &&
        file.read((uint8_t*)&aOY, sizeof(aOY)) &&
        file.read((uint8_t*)&aOZ, sizeof(aOZ)) &&
        file.read((uint8_t*)&gOX, sizeof(gOX)) &&
        file.read((uint8_t*)&gOY, sizeof(gOY)) &&
        file.read((uint8_t*)&gOZ, sizeof(gOZ))) {
        
        file.close();
        Serial.println("Calibration data loaded from LittleFS");
        Serial.printf("Calibration values: aOX=%f, aOY=%f, aOZ=%f, gOX=%f, gOY=%f, gOZ=%f\n", 
                        aOX, aOY, aOZ, gOX, gOY, gOZ);
        return true;
    }
    
    file.close();
    Serial.println("Failed to read calibration data");
    return false;
    }

    void calibrateMPUtoLittleFS(){
        float gyroSumX,gyroSumY,gyroSumZ;
        float accSumX,accSumY,accSumZ;
        float calibCount = 500;
        Serial.println("Calibrating...");

        #if defined(ESP32C3) || defined(ESP32S3)
            for (int i = 0; i < calibCount; i++) {
            //mc.mpu.getMotion6(&mc.ax, &mc.ay, &mc.az, &mc.gx, &mc.gy, &mc.gz);
            mpu.getRealRotation(&gx, &gy, &gz);
            mpu.getRealAcceleration(&ax, &ay, &az);
            gyroSumX += gx;
            gyroSumY += gy;
            gyroSumZ += gz;
            accSumX += ax;
            accSumY += ay;
            accSumZ += az;
            vTaskDelay(10);
            }
        #elif defined(_M5STICKC_H_)
        digitalWrite(10, LOW);
        vTaskDelay(1000);
        digitalWrite(10, HIGH);
        vTaskDelay(100);
        digitalWrite(10, LOW); 
        for(int i = 0; i < calibCount; i++){
            M5.IMU.getGyroData(&gx,&gy,&gz);
            M5.IMU.getAccelData(&ax,&ay,&az);
            gyroSumX += gx;
            gyroSumY += gy;
            gyroSumZ += gz;
            accSumX += ax;
            accSumY += ay;
            accSumZ += az;
            vTaskDelay(10);
            //Serial.printf("%6.2f, %6.2f, %6.2f\r\n", accX, accY, accZ);
        }
        digitalWrite(10, HIGH);
        #endif

        gOX = gyroSumX/calibCount;
        gOY = gyroSumY/calibCount;
        gOZ = gyroSumZ/calibCount;
        aOX = accSumX/calibCount;
        aOY = accSumY/calibCount;
        aOZ = (accSumZ/calibCount) - 1.0;//重力加速度1G、つまりM5ボタンが上向きで行う想定
        //aOZ = (accSumZ/calibCount) + 1.0;//重力加速度1G、つまりM5ボタンが下向きで行う想定
        //aOZ = (accSumZ/calibCount);//
        Serial.println("Calibrating...OK");
        Serial.printf("%3.3f %3.3f %3.3f %3.3f %3.3f %3.3f", aOX, aOY, aOZ, gOX , gOY , gOZ);

        // LittleFSにキャリブレーションデータを保存
        saveCalibrationData();

        // 保存されたデータを検証
        float oldAOX = aOX, oldAOY = aOY, oldAOZ = aOZ, oldGOX = gOX, oldGOY = gOY, oldGOZ = gOZ;
        if (loadCalibrationData()) {
            if (oldAOX == aOX && oldAOY == aOY && oldAOZ == aOZ &&
                oldGOX == gOX && oldGOY == gOY && oldGOZ == gOZ) {
            Serial.println("Calibration data verified successfully");
            } else {
            Serial.println("ERROR: Calibration data verification failed");
            }
        } else {
            Serial.println("ERROR: Failed to load calibration data for verification");
        }

        Serial.printf("Calibration values: aOX=%f, aOY=%f, aOZ=%f, gOX=%f, gOY=%f, gOZ=%f\n", 
                        aOX, aOY, aOZ, gOX, gOY, gOZ);
    }

    void offsetSensorData(){
        ax -= aOX; ay -= aOY; az -= aOZ;
        gx -= gOX; gy -= gOY; gz -= gOZ;
    }


    void m5c_getGyroData(){
        M5.IMU.getGyroData(&gx,&gy,&gz);
    }

    void m5c_getAccelData(){
         M5.IMU.getAccelData(&ax,&ay,&az);
    }

    float* m5c_getAhrsData(){
      //float* quatanion = MahonyAHRSupdateIMU((gx-gOX) * DEG_TO_RAD, (gy-gOY) * DEG_TO_RAD, (gz-gOZ) * DEG_TO_RAD, (ax-aOX), (ay-aOY), (az-aOZ),&rpy[1],&rpy[0],&rpy[2],sampleFreq);
      float* quatanion = MahonyAHRSupdateIMU((gx) * DEG_TO_RAD, (gy) * DEG_TO_RAD, (gz) * DEG_TO_RAD, (ax), (ay), (az),&rpy[1],&rpy[0],&rpy[2],sampleFreq);

      /*
      float roll = 0;
      float pitch = 0;
      float yaw = 0;
      float* quatanion = MahonyAHRSupdateIMU((gx-gOX) * DEG_TO_RAD, (gy-gOY) * DEG_TO_RAD, (gz-gOZ) * DEG_TO_RAD, (ax-aOX), (ay-aOY), (az-aOZ),&pitch,&roll,&yaw,sampleFreq);
      rpy[0] = roll;
      rpy[1] = pitch;
      rpy[2] = yaw;
      */

      //上記引数の表示
      //Serial.printf("%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f",gx,gy,gz,rpy[1],rpy[0],rpy[2]);
      //Serial.printf("%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f",gx,gy,gz,pitch,roll,yaw);

      //Serial.printf("%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f\r\n",rpy[1],rpy[0],rpy[2],pitch,roll,yaw);

      //float* q_array = M5.IMU.getAhrsData(&rpy[1],&rpy[0],&rpy[2],aOX,aOY,aOZ,gOX,gOY,gOZ,); //w,x,y,z
      return  quatanion;
    }



    //==========illumiTracka============
    /*
    void illmiTrack_sensor_read(){
        lit = analogRead(RIGHT_LITTLE);
        rng = analogRead(RIGHT_RING);
        mid = analogRead(RIGHT_MIDDLE);
        idx = analogRead(RIGHT_INDEX);
        

        //float tmb = analogRead(RIGHT_THUMB);
        Serial.printf("%f,%f,%f,%f\r\n",lit,rng,mid,idx);
    }*/

#ifdef ILLUMITRACK_R
    void illmiTrack_switch_sensing(){
        float sensor_threshold = 1;
        
        for(uint8_t i=0; i<sizeof(events_bool); i++){
            //ボタン押下判定
            uint16_t sensor_val = analogRead(button[i]);
            //Serial.println(sensor_val);
            if(prev_button_state[i] == false && sensor_val < sensor_threshold){
                prev_button_state[i] = true;
                events_bool[i] = true;
                //Serial.println("..");
                }
            //ボタン離す判定
            else if(prev_button_state[i] == true && sensor_val >= sensor_threshold){
                prev_button_state[i] = false;
                events_bool[i] = false;
                //Serial.println("!");
            }
        }
    }
#endif

};