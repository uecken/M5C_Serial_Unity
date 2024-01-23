#include "IMUReader.h"

IMUReader::IMUReader(){}

IMUReader::~IMUReader(){}

float IMUReader::unwrappingYaw(float yaw) { // Determines if a phase wrap/unwrap occured and accounts for it
    static int Nwrap;
    static float preYaw = 9999;
    float unwrapYaw;

    if(preYaw == 9999) Nwrap = 0;
    else if ((preYaw  >= 0) && (yaw <= -90)) {
        // Compare the previous theat to the current yaw to see if a wrap occured.
        Nwrap = Nwrap + 1; // If so, add 1 to the number of wraps.
    }
    else if ((preYaw < 0) && (yaw >= 90)) {
        // If an unwrap occured (a phase wrap in the other direction)...
        Nwrap = Nwrap - 1; // ...then subtract 1 from the number of wraps.
    }
    preYaw = yaw; // Store the current yaw for the next cycle
    unwrapYaw = yaw + (float)Nwrap*360.0;
    return unwrapYaw;
}


float IMUReader::unwrappingRoll(float roll,float pitch) {
    static int Nwrap_roll ;
    static float preRoll = 9999;
    float unwrapRoll;
    
    if(preRoll == 9999) Nwrap_roll = 0; //preRoll = roll;
    if( pitch > 60 && pitch <-60){ // pitch±90度付近でRollが180度回転する。この時はwrappingしない。
                        //pitch -60度付近でもRollがジャンプしてしまう
    }else{
        if ((preRoll < -90) && (roll > 90)) {
        Nwrap_roll = Nwrap_roll - 1; 
        }
        else if ((preRoll > 90) && (roll < -90)) {
        Nwrap_roll = Nwrap_roll + 1; 
        }
    }

    preRoll = roll; // Store the current yaw for the next cycle
    unwrapRoll = roll + (float)Nwrap_roll*360.0;
    //Serial.println("unwwapRoll:"+String(unwrapRoll)+","+String(Nwrap_roll));
    return unwrapRoll;
}
