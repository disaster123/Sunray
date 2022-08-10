// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

String GpsWaitFloatOp::name(){
  return "GpsWaitFloat";
}

void GpsWaitFloatOp::begin(){
    // no gps solution
    CONSOLE.println("WARN: no gps solution!");
    stateSensor = SENS_GPS_INVALID;
    gps.reboot();   // try to recover from false GPS fix
    //setOperation(OP_ERROR);
    //buzzer.sound(SND_STUCK, true);          
    
    //linear = 0;
    //angular = 0;      
    //mow = false;
    motor.setLinearAngularSpeed(0,0, false); 
    motor.setMowState(false);    

    retryOperationTime = millis() + 180000;
}


void GpsWaitFloatOp::end(){
}

void GpsWaitFloatOp::run(){
    battery.resetIdle();
    if ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT)){        
        changeOp(*nextOp);
    }     
    if (millis() > retryOperationTime){
        CONSOLE.println("GpsWaitFloatOp timed out - try reboot recovery");
        retryOperationTime = millis() + 180000;
        stateSensor = SENS_GPS_INVALID;
        gps.reboot();
    }
}


