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
bool lift_mode;
bool bumper_mode;

String EscapeReverseOp::name(){
    return "EscapeReverse";
}

void EscapeReverseOp::begin(){
    driveReverseStopTime = millis() + 3000;                           
    bumperReleased = false;
    orig_stateX = stateX;
    orig_stateY = stateY;
    bumper_mode = false;
    lift_mode = false;

    motor.stopImmediately(false);

    if (detectLift()) {
        lift_mode = true;
        motor.setMowState(false);
    }
    if (bumper.obstacle()) {
        bumper_mode = true;
    }

    if (robotShouldRotateLeft()) {
      orig_motion = MOT_LEFT;
    } else if (robotShouldRotateRight()) {
      orig_motion = MOT_RIGHT;
    } else if (robotShouldMoveBackward()) {
      orig_motion = MOT_BACKWARD;
      CONSOLE.println("EscapeReverseOp: BUMPER: motion backward");
    } else {
      orig_motion = MOT_FORWARD;
    }
}


void EscapeReverseOp::end(){
}

void EscapeReverseOp::run(){
    battery.resetIdle();

    // disable mow motor for LIFT only
    if (lift_mode) {
      motor.setMowState(false);
    }

    // drive away in bumper mode as long as bumper was not released and bumper is still triggered
    // or lift_mode
    if ((bumper_mode && !bumperReleased && (bumper.obstacle() || detectLift())) ||
        (lift_mode)) {
      // bumper was not released yet and is still active
      if (orig_motion == MOT_BACKWARD) {
        motor.setLinearAngularSpeed(0.25,0);
      }
      else {
        motor.setLinearAngularSpeed(-0.25,0);
      }
    }

    // if bumper_mode and bumper was not released yet and bumper is released now
    if (bumper_mode && !bumperReleased && !bumper.obstacle() && !detectLift()) {
      CONSOLE.println("BUMPER: now released - set new obstacle position and drive 1s back");
      // bumper is released after obstacle was detected
      bumperReleased = true;
      // overwrite
      orig_stateX = stateX;
      orig_stateY = stateY;

      // from now on drive back 1s
      driveReverseStopTime = millis() + 1000;
    }

    // this code does not work - as it should only work for bumper not for lift
    // we always need to 
    if (bumper_mode && bumperReleased && (bumper.obstacle() || detectLift())) {
      CONSOLE.println("BUMPER: was released but now new obstacle - reset direction and driveReverseStopTime");
      // bumper was released but is pressed now again
      // move again in the other direction - until bumper is released
      // again
      if (orig_motion == MOT_BACKWARD) {
        motor.setLinearAngularSpeed(-0.15,0);
      }
      else {
        motor.setLinearAngularSpeed(0.15,0);
      }
      // this one is updated in every call as long as obstacle is true / triggered
      driveReverseStopTime = millis() + 100;
    }

    // drive back until bumper is no longer triggered or max StopTime
    if (millis() > driveReverseStopTime){
        CONSOLE.println("driveReverseStopTime");
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
        // CONSOLE.println("continue operation with virtual obstacle");
        maps.addObstacle(orig_stateX, orig_stateY, stateDelta, orig_motion);

        float dX = orig_stateX - stateX;
        float dY = orig_stateY - stateY;
        float delta = sqrt( sq(dX) + sq(dY) );    
        if (delta < 0.05){
          CONSOLE.print("EscapeReverseOp: no motion => set motor error! delta: ");
          CONSOLE.println(delta);
          stateSensor = SENS_MOTOR_ERROR;
          changeOp(errorOp);
          return;
        }

        changeOp(*nextOp, false);    // continue current operation
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


