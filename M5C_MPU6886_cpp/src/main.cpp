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
int split(String,char,String*);

float accX = 0.0F,accY = 0.0F,accZ = 0.0F;
float gyroX = 0.0F,gyroY = 0.0F,gyroZ = 0.0F;
float pitch = 0.0F,roll  = 0.0F,yaw   = 0.0F;
float* q_array = new float[4]; //quatanion
float aOX = 0.00, aOY = +0.01, aOZ =  0.07 ;  //-0.00   0.01   0.07 
float gOX = 3.36, gOY = 9.66 , gOZ = 4.11;  //3.36   9.66   4.11
float pO=0 , rO=0 , yO=-8.5, yO2=0;


boolean DEBUG_EEPROM = true;

struct pk{ //8byte (padding 2Byte)
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
pk pk5 = {'p',45,90,'5'};
//pk pk_end = {'0',0,0,'0'}; //Do not use banpei
//pk pk_array[100] = {pk1,pk2,pk3,pk4,pk_end}; //Cannot use vector
//7pk pk_array2[100];
//uint8_t pk_size = sizeof(pk_array)/sizeof(pk_array[0]);
std::vector<pk> pk_vector{};
std::vector<pk> pk_vector_preset1{pk1,pk2,pk3,pk4,pk5};
std::vector<pk> pk_vector_preset_FEZ{pk5};
std::vector<pk> pk_vector_preset_Action{pk5};

//std::vector<pk> pk_vector;
//void flushPKarray(pk*);
//void readPKarray();
const uint16_t EEPROM_SIZE=800;
const uint16_t EEPROM_TEST_START = 0;
const uint16_t EEPROM_TEST_SIZE = 162;
const uint16_t EEPROM_PRES1_START = 162;
const uint16_t EEPROM_PRES1_SIZE = 162;
const uint16_t EEPROM_PRES2_START = 324;
const uint16_t EEPROM_PRES2_SIZE = 162;

//==== Cannot Use EEPROMClass. =====
/*==== The error is [E][EEPROM.cpp:199] commit(): error in write ====
  ==== Use Only EEPROM ==== 
EEPROMClass TEST_ER('eeprom1');
EEPROMClass USER1_ER('eeprom2');
EEPROMClass USER2_ER('eeprom3');
EEPROMClass PRES1_ER('eeprom4');
EEPROMClass PRES2_ER('eeprom5');
*/

void flushPKvector(std::vector<pk>,String);
void readPKvector(String);
void deleteEEPROM(String);
void remove_crlf(std::string&);
void serialprint_pkvector(std::vector<pk>&);

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


  EEPROM.begin(EEPROM_SIZE/5);
  //TEST_ER.begin(EEPROM_SIZE/5); //8Byte * 100/20;
  /*
  USER1_ER.begin(EEPROM_SIZE/5);
  USER2_ER.begin(EEPROM_SIZE/5); 
  PRES1_ER.begin(EEPROM_SIZE/5); 
  PRES2_ER.begin(EEPROM_SIZE/5); 
  */
  //flushPKarray(pk_array);
  //readPKarray();
  
  //flushPKvector(pk_vector);
  //readPKvector();

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

String inputs[10] = {"\0"};
static void ReadSessionLoop(void* arg){
  while (1) {
    uint32_t entryTime = millis();
    if(Serial.available()){//受信データ確認
      M5.Lcd.setCursor(10, 110);
      
      String input_serial = Serial.readStringUntil('/0');
      input_serial.trim(); //remove (" ","\t","\v","\f","\r","\n")
      input_serial_char = input_serial[0];
      Serial.println(input_serial);
      M5.Lcd.println(input_serial);

      int num_length ;
      if(input_serial.indexOf(',')>0){
        num_length = split(input_serial,',',inputs);
        String command = String(inputs[0]);
        String value = String(inputs[1]);
        Serial.println("split:" + command + "," + value);
        if(command == "FLUSH") flushPKvector(pk_vector,value);
        else if(command == "READ") readPKvector(value);
        else if(command == "DELETE") deleteEEPROM(value);
      }else{
        if(input_serial == "FLUSH") flushPKvector(pk_vector,"TEST");
        else if(input_serial == "READ")readPKvector("TEST");
        else if(input_serial == "DELETE") deleteEEPROM("TEST");        
        else if(input_serial == "SHOW") serialprint_pkvector(pk_vector);
      }


      /*
      if(input_serial == "FLUSH") flushPKvector(pk_vector,"TEST");
      else if(input_serial == String("READ")){
        Serial.println("READEEPROOO");
        readPKvector("TEST");
      }
      else if(input_serial == "DELETE") deleteEEPROM("TEST");
      */
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
    if(M5.Axp.GetBtnPress()==2)  //First, read num of element
  int n =0;
  uint8_t num_element;
  EEPROM.get(n,num_element);
  n = sizeof(uint8_t);

  //{ //Instanious Click   // Unexpected "emptyRxFifo(): RxEmpty(2) call on TxBuffer? dq=0" outputs.
      num_powerbtn_click +=1;
    }
*/
    // idle
    int32_t sleep = TASK_SLEEP_BUTTON - (millis() - entryTime);
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}



void flushPKvector(std::vector<pk> pk_vector,String preset_name){
  uint16_t n = 0; uint16_t size=0; 
  if(preset_name=="TEST") n = EEPROM_TEST_START; //size = EEPROM_TEST_SIZE;
  else if(preset_name=="PRES1") n = EEPROM_PRES1_START; //size = EEPROM_PRES1_SIZE;
  else if(preset_name=="PRES2") n = EEPROM_PRES2_START; //size = EEPROM_PRES2_SIZE;

  //==First, write num of element
  uint8_t num_element = pk_vector.size();
  EEPROM.put(n,num_element);
  Serial.println("write:index_byte"+String(n)+",num_element"+String(num_element));
  n += sizeof(uint8_t) * 2;

  //==Second, write pk array;
  for (int i=0 ; i < pk_vector.size(); i++){
    EEPROM.put(n,pk_vector[i]);
    if(DEBUG_EEPROM)Serial.println("write:"+String(n)+","+String(pk_vector[i].hid_input) +","+String(pk_vector[i].rpy_selected));
    n += sizeof(pk);
  }
  EEPROM.commit();
}

void readPKvector(String preset_name){
  uint16_t n = 0; uint16_t size=0; 
  if(preset_name=="TEST") n = EEPROM_TEST_START; //size = EEPROM_TEST_SIZE;
  else if(preset_name=="PRES1") n = EEPROM_PRES1_START; //size = EEPROM_PRES1_SIZE;
  else if(preset_name=="PRES2") n = EEPROM_PRES2_START; //size = EEPROM_PRES2_SIZE;
  else if(preset_name=="ACTION") {pk_vector = pk_vector_preset_Action; return;}

  //==First, read num of element
  //int n = 0;
  uint8_t num_element;
  EEPROM.get(n-2,num_element);
  Serial.println("read:index_byte"+String(n-2)+",num_element"+String(num_element));
  EEPROM.get(n-1,num_element);
  Serial.println("read:index_byte"+String(n-1)+",num_element"+String(num_element));
  EEPROM.get(n,num_element);
  Serial.println("read:index_byte"+String(n)+",num_element"+String(num_element));
  EEPROM.get(n+1,num_element);
  Serial.println("read:index_byte"+String(n+1)+",num_element"+String(num_element));
  EEPROM.get(n+2,num_element);
  Serial.println("read:index_byte"+String(n+2)+",num_element"+String(num_element));
  n += sizeof(uint8_t)*2;
  
  //==Second, read pk array.
  pk pk;
  pk_vector = {};
  for (int i=0; i < num_element; i++){
    EEPROM.get(n,pk);
    pk_vector.push_back(pk);
    //if(pk_vector[i].rpy_selected=='0')break;
    if(DEBUG_EEPROM)Serial.println("read:"+String(n)+","+String(pk_vector[i].hid_input) +","+String(pk_vector[i].rpy_selected));
    n += sizeof(pk);
  }
}

void deleteEEPROM(String preset_name){
  EEPROM.put(0,0);// Set num_element.
  int n = sizeof(uint8_t);

  //Serial.println(String(EEPROM.length())); 0
  //Serial.println(String(EEPROM.length()/sizeof(pk))); 0

  uint16_t size=0; 
  if(preset_name=="TEST") {n = n + EEPROM_TEST_START; size = EEPROM_TEST_SIZE;}
  else if(preset_name=="PRES1"){ n = n + EEPROM_PRES1_START; size = EEPROM_PRES1_SIZE;}
  else if(preset_name=="PRES2"){ n = n + EEPROM_PRES2_START; size = EEPROM_PRES2_SIZE;}

  pk pk_v = {'0',0,0,'0'};
  for (int i=0 ; i < size/sizeof(pk) ; i++){
    EEPROM.put(n,pk_v);
    n += sizeof(pk);
  }
  EEPROM.commit();
  pk_vector = {};
}


/* == Flush and Read by array == 
void flushPKarray(pk *a){
  int n = 0;
  for (int i=0 ; i < SIZE_PK_array; i++){
    EEPROM.put(n,a[i]);
    n += sizeof(pk);
    if(a[i].rpy_selected=='0')break;
  }
  EEPROM.commit();
}

void readPKarray(){
  int n = 0;
  for (int i=0; i <  SIZE_PK_array; i++){
    EEPROM.get(n,pk_array2[i]);
    n += sizeof(pk);
    if(pk_array2[i].rpy_selected=='0')break;
    if(DEBUG_EEPROM)Serial.println("read:"+String(sizeof(pk))+","+String(pk_array2[i].hid_input) +""+String(pk_array2[i].rpy_selected));
  }
}
*/


void serialprint_pkvector(std::vector<pk>& pk_vector){
  for (int i=0 ; i < pk_vector.size(); i++){
    if(DEBUG_EEPROM)Serial.println(String(pk_vector[i].hid_input) +","+String(pk_vector[i].min_angle) + "," + String(pk_vector[i].max_angle)+","+ String(pk_vector[i].rpy_selected));
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

void remove_crlf(std::string& s)
{
    size_t i, j;
    for( i = 0, j = 0; i < s.size(); i++ ){
        if( s[i] != '\r' && s[i] != '\n' ){
            s[j] = s[i];
            j++;
        }
    }
    s.resize(j);
}


// Thanks to https://algorithm.joho.info/arduino/string-split-delimiter/
int split(String data, char delimiter, String *dst){
    //文字列配列の初期化
    for (int j=0; j< sizeof(dst); j++){
      //Serial.println(dst(j));
      dst[j] = {""}; 
    }
    //msgs[3] = {"\0"};
  
    int index = 0;
    int arraySize = (sizeof(data)/sizeof((data)[0]));  
    int datalength = data.length();
    for (int i = 0; i < datalength; i++) {
        char tmp = data.charAt(i);
        if ( tmp == delimiter ) {
            index++;
            if ( index > (arraySize - 1)) return -1;
        }
        else dst[index] += tmp; //区切り文字が来るまで1Byteづつ連結
        //Serial.print("dbg: "+dst[index]);
    }
    return (index + 1);
}
