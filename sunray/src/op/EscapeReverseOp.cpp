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
float orig_stateDelta;
MotType orig_motion;
bool bumperAndLiftReleased;
bool lift_mode;
bool bumper_mode;
float linear;
float distance_to_drive;
bool position_out_of_map;

String EscapeReverseOp::name(){
    return "EscapeReverse";
}

void EscapeReverseOp::begin(){
    driveReverseStopTime = millis() + 3000; // try for max 3s to reach distance_to_drive
    bumperAndLiftReleased = false;
    orig_stateX = stateX;
    orig_stateY = stateY;
    orig_stateDelta = stateDelta;
    bumper_mode = false;
    lift_mode = false;
    position_out_of_map = false;
    linear = 0;
    distance_to_drive = 0.1; // 10cm

    if (detectLift()) {
        lift_mode = true;
        distance_to_drive = 0.2; // 20cm in lift mode
        motor.setMowState(false);
    }
    if (bumper.obstacle()) {
        bumper_mode = true;
    }

    // if none mode is set - may be due to gps no motion - run in bumper mode - now possible due to distance_to_drive
    if (!lift_mode && !bumper_mode) {
      CONSOLE.println("EscapeReverseOp: force bumper_modde");
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

    // this has to run AFTER motion detection above
    motor.stopImmediately(false);
}


void EscapeReverseOp::end(){
}

void EscapeReverseOp::run(){
    battery.resetIdle();

    linear = 0.0;

    // disable mow only for LIFT
    if (lift_mode || DISABLE_MOW_MOTOR_AT_OBSTACLE) {
      motor.setMowState(false);
    }

    float distance_driven = distance(orig_stateX, orig_stateY, stateX, stateY);

    // drive as long as we need to drive
    if (distance_driven < distance_to_drive) {
      // bumper was not released yet and is still active
      if (orig_motion == MOT_BACKWARD) {
        linear = 0.25;
      }
      else {
        linear = -0.25;
      }
      if (lift_mode) {
        linear *= 2;
      }
    }

    // if bumper_mode and bumper was not released yet and bumper is released now
    if (bumper_mode && !bumperAndLiftReleased && !bumper.obstacle() && !detectLift()) {
      CONSOLE.println("BUMPER: released now");
      // bumper is released after obstacle was detected
      bumperAndLiftReleased = true;
    }

    // if ((bumper_mode && bumperAndLiftReleased && (bumper.obstacle() || detectLift())) ||
    //    (lift_mode && !detectLift() && bumper.obstacle())) {
    //   CONSOLE.println("BUMPER/LIFT: was released but now new obstacle - reset direction and driveReverseStopTime");
    //   if (lift_mode) {
    //     bumperAndLiftReleased = true;
    //   }
    //   // bumper was released but is pressed now again
    //   // move again in the other direction - until bumper is released
    //   // again
    //   if (orig_motion == MOT_BACKWARD) {
    //     linear = -0.15;
    //   }
    //   else {
    //     linear = 0.15;
    //   }
    //   // this one is updated in every call as long as obstacle is true / triggered
    //   driveReverseStopTime = millis() + 500;
    // }
   
    // check point but ignore obstacle
    if (maps.checkpoint(stateX, stateY, 0, true)) {
      position_out_of_map = true;
      motor.stopImmediately(false);
    } else {
      motor.setLinearAngularSpeed(linear,0);
    }

    // drive back until bumper is no longer triggered or max StopTime
    if (millis() > driveReverseStopTime || distance_driven >= distance_to_drive || position_out_of_map){

        maps.addObstacle(orig_stateX, orig_stateY, orig_stateDelta, orig_motion);
        if (position_out_of_map) {
          CONSOLE.println("EscapeReverseOp: position out of map - try to continue");
        } else if (millis() > driveReverseStopTime) {
          CONSOLE.println("EscapeReverseOp: driveReverseStopTime - timed out");
          stateSensor = SENS_MOTOR_ERROR;
          changeOp(errorOp);
          return;
        } else {
          CONSOLE.println("EscapeReverseOp: distance driven - OK");

          Point src;
          src.setXY(stateX, stateY);
          if (maps.isPointInsideObstacle(src, -1, -0.04) != -1) {
            // we're OK but now inside obstacle - try to drive 5cm more
            CONSOLE.println("EscapeReverseOp: ut now inside obstacle - try to drive 5cm more");
            distance_to_drive += 0.05;
            return;
          }

        }
        motor.stopImmediately(false);
 
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


