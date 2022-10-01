// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../map.h"
#include "../../helper.h"

float orig_stateX;
float orig_stateY;
MotType orig_motion;
bool bumperReleased;

String EscapeReverseOp::name(){
    return "EscapeReverse";
}

void EscapeReverseOp::begin(){

    // obstacle avoidance
    driveReverseStopTime = millis() + 3000;                           
    bumperReleased = false;

    orig_stateX = stateX;
    orig_stateY = stateY;

    if (robotShouldRotateLeft()) {
      orig_motion = MOT_LEFT;
    } else if (robotShouldRotateRight()) {
      orig_motion = MOT_RIGHT;
    } else if (robotShouldMoveBackward()) {
      orig_motion = MOT_BACKWARD;
    } else {
      orig_motion = MOT_FORWARD;
    }
}


void EscapeReverseOp::end(){
}


void EscapeReverseOp::run(){
    battery.resetIdle();

    if (orig_motion == MOT_BACKWARD) {
      motor.setLinearAngularSpeed(0.15,0);
    }
    else {
      motor.setLinearAngularSpeed(-0.25,0);
    }
    // do not disable mow for obstacle detection
    // motor.setMowState(false);                                        

    if (!bumperReleased && !bumper.obstacle()) {
      // bumper is released after obstacle was detected
      bumperReleased = true;
    }

    if (bumperReleased && bumper.obstacle()) {
      // bumper was released but is pressed now again
      // move again in the other direction
      if (orig_motion == MOT_BACKWARD) {
        motor.setLinearAngularSpeed(-0.1,0);
      }
      else {
        motor.setLinearAngularSpeed(0.1,0);
      }
      driveReverseStopTime = millis() + 200;
    }

    // drive back until bumper is no longer triggered or max StopTime
    if (millis() > driveReverseStopTime){
        CONSOLE.println("driveReverseStopTime or no bumper");
        motor.stopImmediately(false); 
        driveReverseStopTime = 0;
        if (detectLift()) {
            CONSOLE.println("error: after driving reverse lift sensor still active!");
            stateSensor = SENS_LIFT;
            changeOp(errorOp);
            return;
        }
        if (bumper.obstacle()) {
            CONSOLE.println("error: after driving reverse bumper sensor still active!");
	    stateSensor = SENS_BUMPER;
            changeOp(errorOp);
            return;
        }
        if (maps.isDocking()){
            CONSOLE.println("continue docking");
            // continue without obstacles
            changeOp(*nextOp, false);    // continue current operation
        } else {
            CONSOLE.println("continue operation with virtual obstacle");
            maps.addObstacle(stateX, stateY, stateDelta, orig_motion);

            //Point pt;
            //if (!maps.findObstacleSafeMowPoint(pt)){
            //    changeOp(dockOp); // dock if no more (valid) mowing points
            //} else changeOp(*nextOp);    // continue current operation
            changeOp(*nextOp, false);    // continue current operation
        }
    }
}



void EscapeReverseOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void EscapeReverseOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}


