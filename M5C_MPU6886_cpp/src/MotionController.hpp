#ifndef MotionController_H
#define MotionController_H
#include <AccelRingBuffer.hpp>

#include <M5StickC.h>
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

boolean LEFT_DISPLAY = true;


//Mode設定
#define NUM_OF_MODE 4
#define MODE_STREET_FIGHTER_DIST 0
#define MODE_MOTION_CONTROLLER 1  //MOTION INPUT
#define MODE_MOTION_MASSAGE 2  //MOTION MESSAGE
#define MODE_MOUSE 3
#define MODE_MOUSE_KEYBOARD 4
#define MODE_STREET_FIGHTER_RANGE 5 //not used


#define MODE_KEYBOARD 99
//#define MODE_XXXXX 9
#define MODE_TEST_BUTTONB 20


struct SensorData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
};

// ローパスフィルタの係数
const float alpha = 0.9;

//extern BleComboMouse Mouse;
//extern BleComboKeyboard Keyboard;


class MotionController{
    private:

    public:
        //float mouse_speed = 10;
        uint8_t button[3] = {0,36,26};
        //uint8_t button[3] = {0,26};s
        bool prev_button_state[3] = {false,false,false};
        uint8_t pin_LED = 10;
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
        
        float* initial_quat = new float[4];
        float initial_quat_horizontal[4] = {1,0,0,0}; // 寝かせ、ディスプレイ上の初期姿勢のクォータニオン
        float initial_quat_upright[4] = {0.444, 0.469, -0.532, -0.5}; // 右手持ち、ディスプレイ左の初期クォータニオン
        //bool initial_quat_set = false;
        bool set_initial_quaternion = false;
        bool set_initial_quaternion_horizontal = false;
        bool set_initial_quaternion_upright = false;


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
                                            float roll, float pitch, float yaw) {
            // ローパスフィルタを適用してデータを更新
            applyLowPassFilter(sensorDataArray[currentDataIndex_].accelX, accelX);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].accelY, accelY);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].accelZ, accelZ);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].gyroX, gyroX);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].gyroY, gyroY);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].gyroZ, gyroZ);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].roll, roll);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].pitch, pitch);
            applyLowPassFilter(sensorDataArray[currentDataIndex_].yaw, yaw);

            // インデックスを次に進める（ループする場合）
            currentDataIndex_ = (currentDataIndex_ + 1) % dataSize_;
        }


        void begin() {
            M5.begin();  //Init M5StickC Plus.
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
            set_initial_quaternion = true;
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

            while(!bleCombo.isConnected()){
                vTaskDelay(100);
                M5.Lcd.setCursor(5, 35);
                M5.Lcd.print("BLE Connecting....");
            }
            M5.Lcd.print("BLE Connected!      ");
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
        void execSF_HIDInputs(char* inputs,float acc_threshold, AccelRingBuffer* acc_buffer){
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
                if((acc_buffer->getMaxAbsAccel(5)) >= acc_threshold ) acc_flag = true;
                
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
                case MODE_MOTION_CONTROLLER: mode_str  ="MOTION CONTROLLER   "; break;
                case MODE_MOTION_MASSAGE: mode_str     ="MOTION_MASSAGE      "; break;  
                case MODE_MOUSE: mode_str              ="MOUSE               "; break;
                //case MODE_MOUSE_KEYBOARD: mode_str     ="MOUKEY              "; break;
                //case MODE_STREET_FIGHTER_RANGE:mode_str="STREET FIGHTER OLD  "; break;

                default: mode_str="UNKNOWN      "; break;
            }
            M5.Lcd.setCursor(5, 15);
            M5.Lcd.println("Mode:"+mode_str);
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
            M5.Lcd.setCursor(5, 25); 
            M5.Lcd.print("GameMode:"+current_game_mode);
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
            M5.Lcd.setCursor(5, 70);
            M5.Lcd.print("Serial:"+String(serial_ON));
        }

        void serialON(){
            serial_ON = true;
            M5.Lcd.setCursor(5, 70);
            M5.Lcd.print("Serial:"+String(serial_ON));
        }
        void serialOFF(){
            serial_ON = false;
            M5.Lcd.setCursor(5, 70);
            M5.Lcd.print("Serial:"+String(serial_ON));
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



};
#endif