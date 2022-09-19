#include <M5StickCPlus.h>
#include <vector>
#include <EEPROM.h>


#define BTSerial
  #ifdef BTSerial
  #include "BluetoothSerial.h"
  BluetoothSerial bts;
#endif

//Refered to AxisOrange https://github.com/naninunenoy/AxisOrange
#define TASK_DEFAULT_CORE_ID 1
#define TASK_STACK_DEPTH 4096UL
#define TASK_NAME_IMU "IMUTask"
#define TASK_NAME_WRITE_SESSION "WriteSessionTask"
#define TASK_NAME_READ_SESSION "ReadSessionTask"
#define TASK_NAME_BUTTON "ButtonTask"
#define TASK_NAME_HID "HIDTask"
#define TASK_SLEEP_IMU 10 // = 1000[ms] / 100[Hz]
#define TASK_SLEEP_WRITE_SESSION 40 // = 1000[ms] / 25[Hz]
#define TASK_SLEEP_READ_SESSION 100 // = 1000[ms] / 10[Hz]
#define TASK_SLEEP_BUTTON 100 // = 1000[ms] / 10[Hz]
#define MUTEX_DEFAULT_WAIT 10000UL
static void ImuLoop(void* arg);
static void ReadSessionLoop(void* arg);
static void WriteSessionLoop(void* arg);
static void ButtonLoop(void* arg);
static void hidSessionLoop(void* arg);
static SemaphoreHandle_t serWriteMutex = NULL;
static SemaphoreHandle_t imuDataMutex = NULL;
static SemaphoreHandle_t btnDataMutex = NULL;


uint8_t event=0;
#define NO_EVENT 0
#define INPUT_EVENT 1
#define REGISTRATION_ROLL_EVENT 2
#define REGISTRATION_PITCH_EVENT 3
#define xxxxx_EVENT 3

const uint8_t Fs = 50;
const long Ts = (1000/Fs);
const float CONVERT_G_TO_MS2 = 9.80665f;
const boolean left_axis_trans = true; //Unityは左手系、M5Stackは右手系

void calibrateMPU6886();

float accX = 0.0F,accY = 0.0F,accZ = 0.0F;
float gyroX = 0.0F,gyroY = 0.0F,gyroZ = 0.0F;
float pitch = 0.0F,roll  = 0.0F,yaw   = 0.0F;
float* q_array = new float[4]; //quatanion
float aOX = 0.00, aOY = +0.01, aOZ =  0.07 ;  //-0.00   0.01   0.07 
float gOX = 3.36, gOY = 9.66 , gOZ = 4.11;  //3.36   9.66   4.11
float pO=0 , rO=0 , yO=-8.5, yO2=0;




struct pk{ //6byte
  char rpy_selected;  //rpy 1byte
  short min_angle;    // 2byte
  short max_angle;    // 2byte
  char hid_input;   // 1byte
};
// roll -180 ~ +180, pitch -90 ~ +90, yaw -180 ~ +180
pk pk1 = {'r',0,45,'1'};
pk pk2 = {'r',45,90,'2'};
pk pk3 = {'p',0,45,'3'};
pk pk4 = {'p',45,90,'4'};
pk pk_array[4] = {pk1,pk2,pk3,pk4}; //Cannot use vector
//uint8_t pk_size = sizeof(pk_array)/sizeof(pk_array[0]);
std::vector<pk> pk_vector{pk1,pk2};
//std::vector<pk> pk_vector;



char input_serial_char=NULL;
uint8_t num_powerbtn_click=1;
boolean serial_ON = false;

//unsigned long Tcur,Tpre;

void setup() {
  M5.begin();  //Init M5StickC Plus.  初始化 M5StickC Plus
  delay(2000);
  M5.Imu.Init();  //Init IMU.  初始化IMU
  M5.Lcd.setRotation(3);  //Rotate the screen. 将屏幕旋转
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(80, 15); //set the cursor location.  设置光标位置
  M5.Lcd.println("IMU TEST");
  M5.Lcd.setCursor(30, 30);
  M5.Lcd.println("  X       Y       Z");
  M5.Lcd.setCursor(30, 50);
  M5.Lcd.println("  Pitch   Roll    Yaw");

  #ifdef BTSerial
   bts.begin("M5C_BTSerial");
  #endif

  serWriteMutex = xSemaphoreCreateMutex();
  imuDataMutex = xSemaphoreCreateMutex();
  btnDataMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(ImuLoop, TASK_NAME_IMU, TASK_STACK_DEPTH, 
    NULL, 2, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(WriteSessionLoop, TASK_NAME_WRITE_SESSION, TASK_STACK_DEPTH, 
    NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(ReadSessionLoop, TASK_NAME_READ_SESSION, TASK_STACK_DEPTH, 
    NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(ButtonLoop, TASK_NAME_BUTTON, TASK_STACK_DEPTH, 
    NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(hidSessionLoop, TASK_NAME_HID, TASK_STACK_DEPTH, 
    NULL, 1, NULL, TASK_DEFAULT_CORE_ID);

/*
EEPROM.put(0,pk_array);
EEPROM.commit();
pk pk_array;

EEPROM.get(1,pk_array);
int i =0;
Serial.println(String(pk_array[i].rpy_selected) +"," +String(pk_array[i].min_angle) + "," + String(pk_array[i].max_angle) + "," +String(pk_array[i].hid_input));
*/

}

void loop() {
  delay(1);
  }


static void ImuLoop(void* arg) {
  while (1) {
    uint32_t entryTime = millis();
    
    if (xSemaphoreTake(imuDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) {
      M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
      M5.IMU.getAccelData(&accX,&accY,&accZ);
      accX -= aOX; accY -= aOY; accZ -= aOZ;
      gyroX -=gOX; gyroY -=gOY; gyroZ -=gOZ;       
      pitch -=pO;  roll -= rO;  yaw -= yO;//★最小値-188.5。原因はgyroZの補正ができていないからか？一時的に+8.5して補正する。
      q_array = M5.IMU.getAhrsData(&pitch,&roll,&yaw,aOX,aOY,aOZ,gOX,gOY,gOZ);
    }
    xSemaphoreGive(imuDataMutex);
    
    int32_t sleep = TASK_SLEEP_IMU - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void hidSessionLoop(void* arg) {
  while (1) {
    uint32_t entryTime = millis();

    if (xSemaphoreTake(serWriteMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) { // Only for 1 line serial Input
      //姿勢入力判定
      /*
      if(event==INPUT_EVENT){
        if(pk1.min_angle < roll && roll < pk1.max_angle) Serial.println(String("In,")+String(pk1.hid_input)); // NG "In,"+pk1.input. Buffer Over
        else if(pk2.min_angle < pitch && pitch < pk2.max_angle) Serial.println(String("In,")+String(pk2.hid_input));
        M5.Lcd.setCursor(30, 120);
        M5.Lcd.println(pk1.hid_input);
        event = NO_EVENT;
      }*/
      if(event==INPUT_EVENT){
        for(uint8_t i = 0; i < pk_vector.size(); i++){
          int angle = -999;
          switch(pk_vector[i].rpy_selected){
            case 'r' : angle = roll; break;
            case 'p' : angle = pitch; break;
          }
          if(pk_vector[i].min_angle < angle && angle < pk_vector[i].max_angle){
            Serial.println(String("In,")+String(pk_vector[i].hid_input) + ","+String(pk_vector[i].rpy_selected) + "," + String(angle)); // NG "In,"+pk1.input. Buffer Over
            M5.Lcd.setCursor(10,70);
            M5.Lcd.println(String("In,")+String(pk_vector[i].hid_input) + ","+String(pk_vector[i].rpy_selected) + "," + String(angle)); // NG "In,"+pk1.input. Buffer Over

            break;
          }
        }
      }
      else if(event==REGISTRATION_ROLL_EVENT){
        char input_key;
        if(input_serial_char!=NULL) input_key = input_serial_char;
        else input_key = num_powerbtn_click;
        
        pk pk_reg = {'r',roll-22.5,roll+22.5,input_key};
        pk_vector.push_back(pk_reg);
          
        M5.Lcd.println(Serial.readStringUntil('/0'));
        for(uint8_t i = 0; i < pk_vector.size(); i++){
          M5.Lcd.setCursor(10,80+i*7);
          M5.Lcd.println(String(pk_vector[i].rpy_selected) +"," +String(pk_vector[i].min_angle) + "," + String(pk_vector[i].max_angle) + "," +String(pk_vector[i].hid_input));
        }

        input_serial_char = NULL;
        num_powerbtn_click = 1;
      }
      else if(event==REGISTRATION_PITCH_EVENT){
        char input_key = 'p';
        pk pk_reg = {'p',pitch-22.5,pitch+22.5,input_key};
        pk_vector.push_back(pk_reg);
      }
      event = NO_EVENT;
    }
    xSemaphoreGive(serWriteMutex);

    // idle
    int32_t sleep = TASK_SLEEP_WRITE_SESSION - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void WriteSessionLoop(void* arg) {
  while (1) {
    uint32_t entryTime = millis();

    if (xSemaphoreTake(serWriteMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) { // Only for 1 lin serial Input
      if(event==NO_EVENT){
        if(left_axis_trans){//左手系
          if(serial_ON)Serial.printf("%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, -(yaw-yO2),q_array[0],q_array[1],q_array[2],q_array[3]);
          if(serial_ON)bts.printf("%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%1.3f,%1.3f,%1.3f,%1.3f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, -(yaw-yO2),q_array[0],q_array[1],q_array[2],q_array[3]);
        }else{//右手系
          Serial.printf("%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f \r\n", accX, accY, accZ,gyroX, gyroY, gyroZ, pitch, roll, yaw-yO2);
        }
        M5.Lcd.setCursor(30, 60);
        M5.Lcd.println(String(pitch) + " " + String(roll) + " " + String(yaw-yO2));    
        }
    }
    xSemaphoreGive(serWriteMutex);

    // idle
    int32_t sleep = TASK_SLEEP_WRITE_SESSION - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void ReadSessionLoop(void* arg){
  while (1) {
    uint32_t entryTime = millis();
    if(Serial.available()){//受信データ確認
      M5.Lcd.setCursor(10, 110);
      //M5.Lcd.println(Serial.read()); //Byte受信
      String input_serial = Serial.readStringUntil('/0');
      M5.Lcd.println(input_serial);
      input_serial_char = input_serial[0];

    }      
    if(bts.available()){
      M5.Lcd.setCursor(10, 0);
      //M5.Lcd.println(bts.read());　//Byte受信
      M5.Lcd.println(bts.readStringUntil('/0'));
    }
    // idle
    int32_t sleep = TASK_SLEEP_READ_SESSION - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}

static void ButtonLoop(void* arg) {
  uint8_t btnFlag = 0;
  while (1) {
    uint32_t entryTime = millis();

    M5.update();
    if(M5.BtnA.wasReleasefor(1000)){
      if (xSemaphoreTake(imuDataMutex, MUTEX_DEFAULT_WAIT) == pdTRUE) {
        calibrateMPU6886();
        xSemaphoreGive(imuDataMutex);
      }
    }
    else if(M5.BtnA.wasReleasefor(500)){
      if(serial_ON) serial_ON = false;
      else if(serial_ON) serial_ON = true;
    }
    else if(M5.BtnA.wasPressed()){
      Serial.println("button pressed"); 
      yO2 = yaw; //yaw軸の手動補正
    }

    if (M5.BtnB.wasReleasefor(2000)){
      event = REGISTRATION_ROLL_EVENT;
    }
    else if (M5.BtnB.wasReleasefor(1000)){
      event = REGISTRATION_PITCH_EVENT;
    }
    else if(M5.BtnB.wasPressed()){
      event = INPUT_EVENT;
    }
    else{          
    }

/*
    if(M5.Axp.GetBtnPress()==2){ //Instanious Click   // Unexpected "emptyRxFifo(): RxEmpty(2) call on TxBuffer? dq=0" outputs.
      num_powerbtn_click +=1;
    }
*/
    // idle
    int32_t sleep = TASK_SLEEP_BUTTON - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}




void calibrateMPU6886(){
  float gyroSumX,gyroSumY,gyroSumZ;
  float accSumX,accSumY,accSumZ;
  int calibCount = 1000;

  Serial.println("Calibrating...");
  digitalWrite(10, LOW);
  vTaskDelay(2000);
  digitalWrite(10, HIGH);
  vTaskDelay(100);
  digitalWrite(10, LOW); 
  for(int i = 0; i < calibCount; i++){
    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    M5.IMU.getAccelData(&accX,&accY,&accZ);
      gyroSumX += gyroX;
      gyroSumY += gyroY;
      gyroSumZ += gyroZ;
      accSumX += accX;
      accSumY += accY;
      accSumZ += accZ;
      vTaskDelay(2);
      //Serial.printf("%6.2f, %6.2f, %6.2f\r\n", accX, accY, accZ);
  }
  gOX = gyroSumX/calibCount;
  gOY = gyroSumY/calibCount;
  gOZ = gyroSumZ/calibCount;
  aOX = accSumX/calibCount;
  aOY = accSumY/calibCount;
  aOZ = (accSumZ/calibCount) - 1.0;//重力加速度1G、つまりM5ボタンが上向きで行う想定
  //aOZ = (accSumZ/calibCount) + 1.0;//重力加速度1G、つまりM5ボタンが下向きで行う想定
  //aOZ = (accSumZ/calibCount);//
  Serial.println("Calibrating...OK");
  Serial.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", aOX, aOY, aOZ, gOX , gOY , gOZ);
  Serial.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", aOX, aOY, aOZ, gOX , gOY , gOZ);
  /*M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", aOX, aOY, aOZ, gOX , gOY , gOZ);
  */
  digitalWrite(10, HIGH);
}

//void AllSerialOut()