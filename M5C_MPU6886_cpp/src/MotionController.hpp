#ifndef MotionController_H
#define MotionController_H
#include <AccelRingBuffer.hpp>

#include <M5StickC.h>
//#include "MotionController.h"
#include <BleCombo.h>

const uint8_t P2_KEY_UP_ARROW = 'i';
const uint8_t P2_KEY_DOWN_ARROW = 'k';
const uint8_t P2_KEY_LEFT_ARROW = 'j';
const uint8_t P2_KEY_RIGHT_ARROW = 'l';

const String MODE_SF_P1 = "SF_P1";
const String MODE_SF_P2 = "SF_P2";


//extern BleComboMouse Mouse;
//extern BleComboKeyboard Keyboard;


class MotionController{
    private:

    public:
        float mouse_speed = 10;
        uint8_t button[3] = {0,36,26};
        //uint8_t button[3] = {0,26};s
        bool prev_button_state[3] = {false,false,false};
        uint8_t pin_LED = 10;
        String game_modes[2] = {MODE_SF_P1,MODE_SF_P2};
        String current_game_mode = game_modes[0];

        MotionController() {

        }

        ~MotionController() {        
        }


        void begin() {
            M5.begin();  //Init M5StickC Plus.  初始化 M5StickC Plus
            delay(500);
            M5.Imu.Init();  //Init IMU.  初始化IMU
            M5.Lcd.setRotation(3);  //Rotate the screen. 将屏幕旋转
            M5.Lcd.fillScreen(BLACK);
            M5.Lcd.setTextSize(1);
            M5.Lcd.setCursor(5, 5); //set the cursor location.  设置光标位置
            M5.Lcd.println("Street Fighter Controller");
            M5.Lcd.setCursor(30, 30);
            M5.Lcd.println("  X       Y       Z");
            M5.Lcd.setCursor(30, 50);
            M5.Lcd.println("  Pitch   Roll    Yaw");

            pinMode(pin_LED,   OUTPUT); //LED
            pinMode(button[0],   INPUT_PULLUP); //PU 起動モード設定のためか、常にプルアップされており、ソフト制御できない。スイッチでGNDに落として判定する必要あり。
            pinMode(button[1],   INPUT); //3.3V入力をスイッチとして利用可能
            pinMode(button[2],   INPUT_PULLUP); //半PD 電圧が1V程にプルダウン？されている。Pin36を ONにするとノイズが発生し、GNDとの判定で誤判定となる。3.3Vとのショート判定する必要がある。
            //pinMode(button[1],   INPUT_PULLUP);//26

            Keyboard.begin();
            Mouse.begin();

            while(!Keyboard.isConnected()){
                vTaskDelay(100);
                M5.Lcd.setCursor(5, 15); //set the cursor location.  设置光标位置
                M5.Lcd.print("BLE Connecting...");
            }
                M5.Lcd.print("BLE Connected!    ");

        }

        void reConnect() {
            Keyboard.end();
            Mouse.end();
            Keyboard.begin();
            Mouse.begin();
        }


        void inputKey(char key){
            if(Keyboard.isConnected()){
                Keyboard.println(key);   
            }
        }

        void pressKey(char key){
            if(Keyboard.isConnected()){
                Keyboard.press(key);
            }
        }

        void releaseKey(char key){
            if(Keyboard.isConnected()){
                Keyboard.release(key);
            }
        }

        void keyboardReleaseAll(){
            if(Keyboard.isConnected()){
                Keyboard.releaseAll();
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
            
            Mouse.move(x,y);
            return;
        }

        void setMouseSpeed(float speed){
            mouse_speed = speed;
        }

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

        }


        //For Street Fighter
        void execSF_HIDInputs(char* inputs,float acc_threshold, AccelRingBuffer* acc_buffer){
            //文字列終端までfor文を実行する
            int i = 0;
            bool acc_flag = false;


            digitalWriteLED(true);
            
            if(strlen(inputs)<=4){
                delay(100);
            }

            while(inputs[i]!='\0'){
                if((acc_buffer->getMaxAbsAccel(5)) >= acc_threshold ) acc_flag = true;
                
                //過去入力ボタンのリリース
                //(波動拳等のコマンド入力で初期入力の↓がリリースされている必要がある)
                if(i>=2){
                    Keyboard.release(inputs[i-2]);
                    vTaskDelay(20);
       
                }

                //最終入力値以外の通常入力
                if(inputs[i+1]!='\0'){
                    Keyboard.press(inputs[i]);
                    vTaskDelay(30);
                }
                //最終入力値は一定以上の閾値が観測された場合に実行する
                else if(inputs[i+1]=='\0'){
                    if(acc_flag){
                        digitalWriteLED(false);
                        Keyboard.press(inputs[i]);
                        acc_flag = false;
                    }
                    vTaskDelay(30);
                    digitalWriteLED(true); 
                }
                i++;
            }


            if(inputs[i]=='\0'){
                vTaskDelay(50);
                Keyboard.releaseAll();
                digitalWriteLED(false);
            }

        }

        void changeGameMode()
        {
            if(current_game_mode == game_modes[0]){
                current_game_mode = game_modes[1];
            }else{
                current_game_mode = game_modes[0];
            }
            M5.Lcd.setCursor(5, 15); 
            M5.Lcd.print("GameMode:"+current_game_mode);
        }


        void SF_hadoken(){
                Keyboard.press(KEY_DOWN_ARROW);
                vTaskDelay(50);
                Keyboard.press(KEY_RIGHT_ARROW);
                vTaskDelay(50);

                Keyboard.release(KEY_DOWN_ARROW);
                Keyboard.press(KEY_RIGHT_ARROW);
                vTaskDelay(50);

                Keyboard.press('a');
                //TaskDelay(50);
        }

        void digitalWriteLED(bool state){
            if(state==true){digitalWrite(pin_LED, LOW);} //ON
            else{digitalWrite(pin_LED, HIGH);} //OFF
        }
};
#endif